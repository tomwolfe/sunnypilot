#pragma once

#include <vector>
#include <string>

// Define OPENPILOT_RAYLIB before including raylib to prevent enum conflicts
#define OPENPILOT_RAYLIB
#include "third_party/raylib/include/raylib.h"

// Temporarily undefine conflicting macros to avoid capnp conflicts
#ifdef RED
#undef RED
#endif
#ifdef GREEN
#undef GREEN
#endif
#ifdef YELLOW
#undef YELLOW
#endif
#ifdef BLUE
#undef BLUE
#endif
#ifdef MAGENTA
#undef MAGENTA
#endif
#ifdef CYAN
#undef CYAN
#endif
#ifdef WHITE
#undef WHITE
#endif
#ifdef BLACK
#undef BLACK
#endif
#ifdef GRAY
#undef GRAY
#endif
#ifdef LIGHTGRAY
#undef LIGHTGRAY
#endif
#ifdef DARKGRAY
#undef DARKGRAY
#endif

#include "tools/cabana/streams/replaystream.h"

// Define color constants for use in the file
#define RED ((Color){230, 41, 55, 255})
#define GREEN ((Color){0, 228, 48, 255})
#define YELLOW ((Color){253, 249, 0, 255})
#define BLUE ((Color){0, 121, 241, 255})
#define MAGENTA ((Color){255, 0, 255, 255})
#define CYAN ((Color){0, 255, 255, 255})
#define WHITE ((Color){255, 255, 255, 255})
#define BLACK ((Color){0, 0, 0, 255})
#define GRAY ((Color){129, 129, 129, 255})
#define LIGHTGRAY ((Color){200, 200, 200, 255})
#define DARKGRAY ((Color){80, 80, 80, 255})

// Data structure for segment info
struct SegmentInfo {
    int segment_number;
    bool has_rlog;
    bool has_fcam;
    bool has_ecam; 
    bool has_dcam;
    bool has_qlog;
    bool has_qcam;
    std::string rlog_status;
    std::string fcam_status;
    std::string ecam_status;
    std::string dcam_status;
    std::string qlog_status;
    std::string qcam_status;
};

// Raylib-based RouteInfoDlg
class RouteInfoDlg {
public:
    RouteInfoDlg(void* parent = nullptr);
    ~RouteInfoDlg();
    
    void show();
    void hide();
    bool isVisible() const { return visible; }
    void update();
    void render(const Rectangle& bounds);
    void handleInput();

    // Callback for when a segment should be sought to
    std::function<void(double)> onSeekToSegment;

private:
    void loadSegmentInfo();
    void drawTable(const Rectangle& bounds);
    int getRowAtPosition(float y, const Rectangle& bounds) const;

    // UI state
    bool visible = false;
    Rectangle windowBounds;
    std::string routeName;
    std::vector<SegmentInfo> segments;
    
    // UI elements bounds
    Rectangle tableBounds;
    
    // UI state
    int selectedRow = -1;
    float scroll_offset_ = 0.0f;
};

extern RouteInfoDlg* routeInfoDlg;