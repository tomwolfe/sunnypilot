"""
Real-Time Uncertainty Estimation for E2E Driving
=================================================

This module implements uncertainty estimation for the E2E model using:
1. Monte Carlo Dropout (MC Dropout)
2. Laplace Approximation
3. Ensemble-based uncertainty

Key Features:
- Real-time confidence scoring
- Uncertainty-aware planning
- Automatic camera sampling adjustment based on uncertainty
- Gain sharpening during high uncertainty conditions
"""

import numpy as np
from dataclasses import dataclass
from typing import Optional
from collections import deque


@dataclass
class UncertaintyOutput:
    """Output from uncertainty estimation"""
    epistemic_uncertainty: float  # Model uncertainty (reducible with more data)
    aleatoric_uncertainty: float  # Data uncertainty (inherent noise)
    total_uncertainty: float
    confidence_score: float
    is_high_uncertainty: bool
    recommended_action: str  # "normal", "cautious", "fallback"


class MCDropoutEstimator:
    """
    Monte Carlo Dropout Uncertainty Estimator

    Uses dropout at inference time to generate multiple stochastic forward passes.
    The variance across passes estimates epistemic (model) uncertainty.

    Advantages:
    - No retraining required
    - Computationally efficient
    - Works with existing neural networks
    """

    def __init__(self,
                 num_samples: int = 10,
                 dropout_rate: float = 0.1,
                 feature_dim: int = 256):
        self.num_samples = num_samples
        self.dropout_rate = dropout_rate
        self.feature_dim = feature_dim

        self._dropout_masks: list[np.ndarray] = []
        self._generate_dropout_masks()

        self._uncertainty_history = deque(maxlen=50)
        self._running_mean = 0.0
        self._running_var = 1.0

    def _generate_dropout_masks(self):
        """Generate dropout masks for MC sampling"""
        self._dropout_masks = []
        for _ in range(self.num_samples):
            mask = (np.random.rand(self.feature_dim) > self.dropout_rate).astype(np.float32)
            mask /= (1.0 - self.dropout_rate)  # Scale to maintain expected value
            self._dropout_masks.append(mask)

    def estimate_uncertainty(self,
                            forward_pass_fn,
                            input_features: np.ndarray) -> UncertaintyOutput:
        """
        Estimate uncertainty using MC Dropout

        Args:
            forward_pass_fn: Function that takes (features, dropout_mask) and returns output
            input_features: Input feature vector

        Returns:
            UncertaintyOutput with epistemic and aleatoric components
        """
        outputs = []

        # Multiple stochastic forward passes
        for mask in self._dropout_masks:
            output = forward_pass_fn(input_features, mask)
            outputs.append(output)

        outputs = np.array(outputs)

        # Mean prediction
        mean_output = np.mean(outputs, axis=0)

        # Epistemic uncertainty (variance across dropout samples)
        epistemic = np.var(outputs, axis=0)
        epistemic_scalar = float(np.mean(epistemic))

        # Update running statistics
        self._update_running_stats(epistemic_scalar)

        # Normalize uncertainty
        normalized_epistemic = self._normalize_uncertainty(epistemic_scalar)

        # Confidence score (inverse of uncertainty)
        confidence = 1.0 - np.clip(normalized_epistemic, 0.0, 1.0)

        # Determine recommended action
        is_high, recommended_action = self._determine_action(normalized_epistemic)

        return UncertaintyOutput(
            epistemic_uncertainty=normalized_epistemic,
            aleatoric_uncertainty=0.0,  # Would need heteroscedastic loss to estimate
            total_uncertainty=normalized_epistemic,
            confidence_score=float(confidence),
            is_high_uncertainty=is_high,
            recommended_action=recommended_action
        )

    def _update_running_stats(self, uncertainty: float):
        """Update running mean and variance of uncertainty"""
        alpha = 0.1
        self._running_mean = (1 - alpha) * self._running_mean + alpha * uncertainty
        self._running_var = (1 - alpha) * self._running_var + alpha * (uncertainty - self._running_mean) ** 2

    def _normalize_uncertainty(self, uncertainty: float) -> float:
        """Normalize uncertainty using running statistics"""
        std = np.sqrt(self._running_var + 1e-6)
        normalized = (uncertainty - self._running_mean) / (std + 1e-6)
        # Sigmoid normalization to [0, 1]
        normalized = 1.0 / (1.0 + np.exp(-normalized))
        return float(np.clip(normalized, 0.0, 1.0))

    def _determine_action(self, normalized_uncertainty: float) -> tuple[bool, str]:
        """Determine recommended action based on uncertainty"""
        if normalized_uncertainty > 0.8:
            return True, "fallback"
        elif normalized_uncertainty > 0.6:
            return True, "cautious"
        else:
            return False, "normal"

    def reset(self):
        """Reset uncertainty statistics"""
        self._uncertainty_history.clear()
        self._running_mean = 0.0
        self._running_var = 1.0


class LaplaceApproximationEstimator:
    """
    Laplace Approximation for Uncertainty Estimation

    Approximates the posterior distribution of network weights using a Gaussian
    centered at the MAP estimate. The inverse Hessian provides uncertainty estimates.

    Advantages:
    - More accurate than MC Dropout
    - Provides both mean and variance predictions
    - Well-founded in Bayesian statistics

    Disadvantages:
    - Requires computing/approximating Hessian
    - More computationally expensive
    """

    def __init__(self,
                 feature_dim: int = 256,
                 output_dim: int = 4,
                 prior_precision: float = 1.0):
        self.feature_dim = feature_dim
        self.output_dim = output_dim
        self.prior_precision = prior_precision

        # Diagonal approximation of Hessian (for efficiency)
        self._hessian_diag = np.ones(feature_dim * output_dim, dtype=np.float32) * prior_precision
        self._gradient_buffer = np.zeros(feature_dim * output_dim, dtype=np.float32)

        self._uncertainty_history = deque(maxlen=50)
        self._running_mean = 0.0
        self._running_var = 1.0

    def update_hessian_diagonal(self,
                               features: np.ndarray,
                               predictions: np.ndarray,
                               targets: Optional[np.ndarray] = None):
        """
        Update diagonal Hessian approximation using Gauss-Newton

        Args:
            features: Input features [batch, feature_dim]
            predictions: Model predictions [batch, output_dim]
            targets: Optional targets for loss computation
        """
        batch_size = features.shape[0]

        # Compute gradient outer product approximation
        for i in range(batch_size):
            feat = features[i]
            pred = predictions[i]

            # Gradient of MSE loss w.r.t. weights (simplified)
            grad = np.outer(feat, pred).flatten()

            # Exponential moving average of Hessian diagonal
            alpha = 0.01
            self._hessian_diag = (1 - alpha) * self._hessian_diag + alpha * (grad ** 2 + self.prior_precision)

    def estimate_uncertainty(self,
                            features: np.ndarray,
                            weights: np.ndarray) -> UncertaintyOutput:
        """
        Estimate uncertainty using Laplace approximation

        Args:
            features: Input features
            weights: Network weights (flattened)

        Returns:
            UncertaintyOutput
        """
        # Posterior precision = Hessian + prior
        posterior_precision = self._hessian_diag + self.prior_precision

        # Posterior variance (diagonal)
        posterior_var = 1.0 / (posterior_precision + 1e-8)

        # Predictive variance at input features
        # Var[f(x)] ≈ grad[f(x)]^T * Var[w] * grad[f(x)]
        feature_var = np.var(features, axis=0) if features.ndim > 1 else np.ones_like(features)
        predictive_var = np.sum(posterior_var * feature_var)

        epistemic_scalar = float(predictive_var / len(posterior_var))

        # Normalize
        self._update_running_stats(epistemic_scalar)
        normalized = self._normalize_uncertainty(epistemic_scalar)

        # Confidence and action
        confidence = 1.0 - np.clip(normalized, 0.0, 1.0)
        is_high, recommended_action = self._determine_action(normalized)

        return UncertaintyOutput(
            epistemic_uncertainty=normalized,
            aleatoric_uncertainty=0.0,
            total_uncertainty=normalized,
            confidence_score=float(confidence),
            is_high_uncertainty=is_high,
            recommended_action=recommended_action
        )

    def _update_running_stats(self, uncertainty: float):
        """Update running statistics"""
        alpha = 0.1
        self._running_mean = (1 - alpha) * self._running_mean + alpha * uncertainty
        self._running_var = (1 - alpha) * self._running_var + alpha * (uncertainty - self._running_mean) ** 2

    def _normalize_uncertainty(self, uncertainty: float) -> float:
        """Normalize uncertainty"""
        std = np.sqrt(self._running_var + 1e-6)
        normalized = (uncertainty - self._running_mean) / (std + 1e-6)
        normalized = 1.0 / (1.0 + np.exp(-normalized))
        return float(np.clip(normalized, 0.0, 1.0))

    def _determine_action(self, normalized_uncertainty: float) -> tuple[bool, str]:
        """Determine recommended action"""
        if normalized_uncertainty > 0.8:
            return True, "fallback"
        elif normalized_uncertainty > 0.6:
            return True, "cautious"
        else:
            return False, "normal"


class EnsembleUncertaintyEstimator:
    """
    Ensemble-based Uncertainty Estimator

    Uses an ensemble of models trained with different initializations.
    Disagreement between ensemble members indicates uncertainty.

    Advantages:
    - Most reliable uncertainty estimates
    - Captures multi-modal predictions
    - Robust to distributional shift

    Disadvantages:
    - Requires training multiple models
    - Higher memory footprint
    - Slower inference (N forward passes)
    """

    def __init__(self,
                 num_ensemble: int = 5,
                 feature_dim: int = 256):
        self.num_ensemble = num_ensemble
        self.feature_dim = feature_dim

        self._ensemble_weights: list[np.ndarray] = []
        self._initialize_ensemble()

        self._uncertainty_history = deque(maxlen=50)
        self._running_mean = 0.0
        self._running_var = 1.0

    def _initialize_ensemble(self):
        """Initialize ensemble with diverse weights"""
        self._ensemble_weights = []
        for _ in range(self.num_ensemble):
            # Diverse initialization
            weights = np.random.randn(self.feature_dim).astype(np.float32) * 0.01
            self._ensemble_weights.append(weights)

    def estimate_uncertainty(self,
                            forward_pass_fns: list,
                            input_features: np.ndarray) -> UncertaintyOutput:
        """
        Estimate uncertainty using ensemble disagreement

        Args:
            forward_pass_fns: List of forward pass functions (one per ensemble member)
            input_features: Input features

        Returns:
            UncertaintyOutput
        """
        outputs = []

        # Forward pass through each ensemble member
        for i, fn in enumerate(forward_pass_fns):
            if i < len(self._ensemble_weights):
                output = fn(input_features, self._ensemble_weights[i])
                outputs.append(output)

        if len(outputs) < 2:
            return UncertaintyOutput(
                epistemic_uncertainty=0.5,
                aleatoric_uncertainty=0.0,
                total_uncertainty=0.5,
                confidence_score=0.5,
                is_high_uncertainty=False,
                recommended_action="normal"
            )

        outputs = np.array(outputs)

        # Mean prediction
        mean_output = np.mean(outputs, axis=0)

        # Epistemic uncertainty (ensemble disagreement)
        ensemble_var = np.var(outputs, axis=0)
        epistemic_scalar = float(np.mean(ensemble_var))

        # Aleatoric uncertainty (average predicted variance if available)
        # For now, use mean of individual predictions as proxy
        aleatoric_scalar = float(np.mean(np.abs(outputs - mean_output)))

        # Total uncertainty
        total_uncertainty = epistemic_scalar + 0.5 * aleatoric_scalar

        # Normalize
        self._update_running_stats(epistemic_scalar)
        normalized = self._normalize_uncertainty(epistemic_scalar)

        # Confidence and action
        confidence = 1.0 - np.clip(normalized, 0.0, 1.0)
        is_high, recommended_action = self._determine_action(normalized)

        return UncertaintyOutput(
            epistemic_uncertainty=normalized,
            aleatoric_uncertainty=float(np.clip(aleatoric_scalar, 0.0, 1.0)),
            total_uncertainty=float(np.clip(total_uncertainty / 2.0, 0.0, 1.0)),
            confidence_score=float(confidence),
            is_high_uncertainty=is_high,
            recommended_action=recommended_action
        )

    def _update_running_stats(self, uncertainty: float):
        """Update running statistics"""
        alpha = 0.1
        self._running_mean = (1 - alpha) * self._running_mean + alpha * uncertainty
        self._running_var = (1 - alpha) * self._running_var + alpha * (uncertainty - self._running_mean) ** 2

    def _normalize_uncertainty(self, uncertainty: float) -> float:
        """Normalize uncertainty"""
        std = np.sqrt(self._running_var + 1e-6)
        normalized = (uncertainty - self._running_mean) / (std + 1e-6)
        normalized = 1.0 / (1.0 + np.exp(-normalized))
        return float(np.clip(normalized, 0.0, 1.0))

    def _determine_action(self, normalized_uncertainty: float) -> tuple[bool, str]:
        """Determine recommended action"""
        if normalized_uncertainty > 0.8:
            return True, "fallback"
        elif normalized_uncertainty > 0.6:
            return True, "cautious"
        else:
            return False, "normal"


class UncertaintyAwareController:
    """
    Uncertainty-Aware E2E Controller

    Integrates uncertainty estimation into the E2E control loop:
    1. Estimates uncertainty in real-time
    2. Adjusts camera sampling frequency based on uncertainty
    3. Sharpens feedback gains during high uncertainty
    4. Triggers fallback when uncertainty exceeds threshold
    """

    UNCERTAINTY_HIGH_THRESHOLD = 0.7
    UNCERTAINTY_CRITICAL_THRESHOLD = 0.9

    def __init__(self,
                 method: str = 'mc_dropout',
                 num_samples: int = 10,
                 feature_dim: int = 256):
        """
        Initialize uncertainty-aware controller

        Args:
            method: Uncertainty estimation method ('mc_dropout', 'laplace', 'ensemble')
            num_samples: Number of MC samples / ensemble members
            feature_dim: Feature dimension
        """
        self.method = method

        if method == 'mc_dropout':
            self.estimator = MCDropoutEstimator(
                num_samples=num_samples,
                feature_dim=feature_dim
            )
        elif method == 'laplace':
            self.estimator = LaplaceApproximationEstimator(
                feature_dim=feature_dim
            )
        elif method == 'ensemble':
            self.estimator = EnsembleUncertaintyEstimator(
                num_ensemble=num_samples,
                feature_dim=feature_dim
            )
        else:
            raise ValueError(f"Unknown uncertainty method: {method}")

        self._last_uncertainty = UncertaintyOutput(
            epistemic_uncertainty=0.0,
            aleatoric_uncertainty=0.0,
            total_uncertainty=0.0,
            confidence_score=1.0,
            is_high_uncertainty=False,
            recommended_action="normal"
        )

        self._camera_sampling_rate = 20  # Hz
        self._base_sampling_rate = 20
        self._gain_sharpening_factor = 1.0

    def estimate(self,
                forward_pass_fn,
                input_features: np.ndarray) -> UncertaintyOutput:
        """
        Estimate uncertainty and update controller state

        Args:
            forward_pass_fn: Forward pass function
            input_features: Input features

        Returns:
            UncertaintyOutput
        """
        self._last_uncertainty = self.estimator.estimate_uncertainty(
            forward_pass_fn,
            input_features
        )

        # Adjust camera sampling rate
        self._adjust_camera_sampling()

        # Adjust control gains
        self._adjust_gains()

        return self._last_uncertainty

    def _adjust_camera_sampling(self):
        """Adjust camera sampling frequency based on uncertainty"""
        uncertainty = self._last_uncertainty.total_uncertainty

        if uncertainty > self.UNCERTAINTY_CRITICAL_THRESHOLD:
            # Critical: Maximize sampling
            self._camera_sampling_rate = 30  # Max for TICI
        elif uncertainty > self.UNCERTAINTY_HIGH_THRESHOLD:
            # High uncertainty: Increase sampling
            self._camera_sampling_rate = 25
        else:
            # Normal: Base rate
            self._camera_sampling_rate = self._base_sampling_rate

    def _adjust_gains(self):
        """Adjust control gains based on uncertainty"""
        uncertainty = self._last_uncertainty.total_uncertainty

        if uncertainty > self.UNCERTAINTY_CRITICAL_THRESHOLD:
            # Critical: Sharpen gains significantly
            self._gain_sharpening_factor = 1.5
        elif uncertainty > self.UNCERTAINTY_HIGH_THRESHOLD:
            # High: Moderate gain sharpening
            self._gain_sharpening_factor = 1.2
        else:
            # Normal: Base gains
            self._gain_sharpening_factor = 1.0

    def get_camera_sampling_rate(self) -> int:
        """Get recommended camera sampling rate"""
        return self._camera_sampling_rate

    def get_gain_sharpening_factor(self) -> float:
        """Get gain sharpening factor"""
        return self._gain_sharpening_factor

    def should_fallback(self) -> bool:
        """Check if system should fall back to classical control"""
        return self._last_uncertainty.recommended_action == "fallback"

    def get_uncertainty(self) -> UncertaintyOutput:
        """Get latest uncertainty estimate"""
        return self._last_uncertainty

    def reset(self):
        """Reset uncertainty estimator"""
        self.estimator.reset()
        self._camera_sampling_rate = self._base_sampling_rate
        self._gain_sharpening_factor = 1.0
