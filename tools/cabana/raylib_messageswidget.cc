#include "raylib_messageswidget.h"
#include "tools/cabana/utils/eventmanager.h"
#include "tools/cabana/utils/logger.h"
#include <algorithm>
#include <sstream>
#include <iomanip>

MessageListModel::MessageListModel() {
  Logger::instance().info("MESSAGES_MODEL", "MessageListModel initialized");
  // Initialize with some sample messages for now
  // In the real implementation, this will be populated from the stream
}

void MessageListModel::sort(int column, bool ascending) {
  sort_column = column;
  sort_order = ascending;
  Logger::instance().debug("MESSAGES_MODEL", "Sorting by column: ", column, ", ascending: ", ascending);
  sortItems(items_);
}

void MessageListModel::setFilterStrings(const std::map<int, std::string> &filters) {
  filters_ = filters;
  filterAndSort();
}

void MessageListModel::showInactiveMessages(bool show) {
  show_inactive_messages = show;
  filterAndSort();
}

void MessageListModel::msgsReceived(const std::set<MessageId> *new_msgs, bool has_new_ids) {
  if (!new_msgs) {
    Logger::instance().warning("MESSAGES_MODEL", "msgsReceived called with null new_msgs");
    return;
  }

  Logger::instance().debug("MESSAGES_MODEL", "Received new messages: ", new_msgs->size(), ", has_new_ids: ", has_new_ids);

  // In a real implementation, this would update the model with new messages
  if (has_new_ids) {
    // Add new sample messages - in real implementation, actual messages would be added
    for (const auto &msg_id : *new_msgs) {
      // Find or create an item for this message ID
      bool found = false;
      for (auto &item : items_) {
        if (item.id == msg_id) {
          found = true;
          break;
        }
      }

      if (!found) {
        Item newItem;
        newItem.id = msg_id;
        newItem.name = "MSG_" + std::to_string(msg_id.address);
        newItem.node = "CAN";
        items_.push_back(newItem);
      }
    }

    filterAndSort();
  }
}

bool MessageListModel::filterAndSort() {
  Logger::instance().debug("MESSAGES_MODEL", "Filtering and sorting items, total: ", items_.size());

  // Filter and sort items based on current filters and sorting settings
  filtered_items_.clear();

  for (const auto &item : items_) {
    if (match(item)) {
      filtered_items_.push_back(item);
    }
  }

  sortItems(filtered_items_);
  Logger::instance().debug("MESSAGES_MODEL", "Filtered items: ", filtered_items_.size());
  return true;
}

void MessageListModel::dbcModified() {
  // Handle DBC modification updates
  Logger::instance().info("MESSAGES_MODEL", "DBC modified, refiltering");
  filterAndSort();
}

void MessageListModel::sortItems(std::vector<Item> &items) {
  std::sort(items.begin(), items.end(), [this](const Item &a, const Item &b) {
    bool result = false;
    
    switch (sort_column) {
      case NAME:
        result = a.name < b.name;
        break;
      case SOURCE:
        result = a.id.src < b.id.src;
        break;
      case ADDRESS:
        result = a.id.address < b.id.address;
        break;
      case NODE:
        result = a.node < b.node;
        break;
      default:
        result = a.id.address < b.id.address; // Default to address
        break;
    }
    
    return sort_order ? result : !result;
  });
}

bool MessageListModel::match(const Item &item) {
  // Check if item matches all active filters
  for (const auto &[column, filter] : filters_) {
    if (filter.empty()) continue;
    
    std::string value;
    switch (column) {
      case NAME:
        value = item.name;
        break;
      case SOURCE:
        value = std::to_string(item.id.src);
        break;
      case ADDRESS:
        value = "0x" + std::to_string(item.id.address);
        break;
      case NODE:
        value = item.node;
        break;
      default:
        continue;
    }
    
    // Simple case-insensitive substring match
    std::string lower_value = value;
    std::string lower_filter = filter;
    std::transform(lower_value.begin(), lower_value.end(), lower_value.begin(), ::tolower);
    std::transform(lower_filter.begin(), lower_filter.end(), lower_filter.begin(), ::tolower);
    
    if (lower_value.find(lower_filter) == std::string::npos) {
      return false;
    }
  }
  
  return true;
}

// MessagesWidget implementation
MessagesWidget::MessagesWidget(void* parent) {
  model_ = std::make_unique<MessageListModel>();
  // Initialize with empty data - will be populated from stream
}

MessagesWidget::~MessagesWidget() {
  disconnectFromStream();
}

void MessagesWidget::connectToStream(AbstractStream* stream) {
  if (!stream) {
    Logger::instance().error("MESSAGES_WIDGET", "connectToStream called with null stream");
    return;
  }

  Logger::instance().info("MESSAGES_WIDGET", "Connecting to stream");
  disconnectFromStream(); // Clean up any existing connections

  // Connect to the stream's native signals
  stream->msgsReceived_signal.connect([this, stream](const std::set<MessageId>* new_msgs, bool has_new_ids) {
    if (has_new_ids && new_msgs) {
      Logger::instance().debug("MESSAGES_WIDGET", "Stream reported new messages, count: ", new_msgs->size());

      // First, let the model know about new message IDs
      model_->msgsReceived(new_msgs, has_new_ids);

      // Update message details with latest data from the stream
      for (const auto& msg_id : *new_msgs) {
        const auto& last_msg = stream->lastMessage(msg_id);
        if (!last_msg.dat.empty()) {
          // Update the corresponding item with real data
          bool item_found = false;
          for (auto& item : model_->items_) {
            if (item.id == msg_id) {
              // Format the data as hex string
              item.data.clear();
              for (size_t i = 0; i < last_msg.dat.size(); ++i) {
                char buf[4];
                snprintf(buf, sizeof(buf), "%02X ", last_msg.dat[i]);
                item.data += std::string(buf);
              }
              if (!item.data.empty()) {
                item.data.pop_back(); // Remove trailing space
              }
              item.frequency = last_msg.freq;
              item.count = last_msg.count;
              item_found = true;
              break;
            }
          }

          // If the item doesn't exist in our model, create it
          if (!item_found) {
            MessageListModel::Item newItem;
            newItem.id = msg_id;
            newItem.name = "MSG_" + std::to_string(msg_id.address);
            newItem.node = "CAN";

            // Format the data as hex string
            newItem.data.clear();
            for (size_t i = 0; i < last_msg.dat.size(); ++i) {
              char buf[4];
              snprintf(buf, sizeof(buf), "%02X ", last_msg.dat[i]);
              newItem.data += std::string(buf);
            }
            if (!newItem.data.empty()) {
              newItem.data.pop_back(); // Remove trailing space
            }
            newItem.frequency = last_msg.freq;
            newItem.count = last_msg.count;

            model_->items_.push_back(newItem);
          }
        }
      }

      updateTitle();
    }
  });

  // Also connect to update the message details periodically
  stream->eventsMerged_signal.connect([this, stream](const MessageEventsMap& events_map) {
    Logger::instance().debug("MESSAGES_WIDGET", "Events merged, updating display");
    // Update existing items with latest data
    for (auto& item : model_->items_) {
      const auto& last_msg = stream->lastMessage(item.id);
      if (!last_msg.dat.empty()) {
        // Format the data as hex string
        item.data.clear();
        for (size_t i = 0; i < last_msg.dat.size(); ++i) {
          char buf[4];
          snprintf(buf, sizeof(buf), "%02X ", last_msg.dat[i]);
          item.data += std::string(buf);
        }
        if (!item.data.empty()) {
          item.data.pop_back(); // Remove trailing space
        }
        item.frequency = last_msg.freq;
        item.count = last_msg.count;
      }
    }
  });

  connected_to_stream = true;
  Logger::instance().info("MESSAGES_WIDGET", "Successfully connected to stream");
}

void MessagesWidget::disconnectFromStream() {
  Logger::instance().info("MESSAGES_WIDGET", "Disconnecting from stream");
  if (msg_received_subscription_id) {
    EventManager::instance().unsubscribe(msg_received_subscription_id);
    msg_received_subscription_id = 0;
  }
  connected_to_stream = false;
  Logger::instance().info("MESSAGES_WIDGET", "Successfully disconnected from stream");
}

void MessagesWidget::update() {
  // Update internal state
  handleScrolling();
}

void MessagesWidget::render(const Rectangle& bounds) {
  if (!is_visible) return;

  bounds_ = bounds;

  // Draw panel background
  DrawRectangleRec(bounds, Color{240, 240, 240, 255}); // Light gray
  DrawRectangleLines(bounds.x, bounds.y, bounds.width, bounds.height, (Color){211, 211, 211, 255}); // LIGHTGRAY in RGB

  // Draw title
  DrawText(title.c_str(), bounds.x + 10, bounds.y + 5, 14, (Color){64, 64, 64, 255}); // DARKGRAY in RGB

  // Draw header row
  float header_height = 20;
  DrawRectangle(bounds.x, bounds.y + 20, bounds.width, header_height, Color{200, 200, 200, 255});

  // Draw header labels
  const char* headers[] = {"Name", "Source", "Address", "Node", "Freq", "Count", "Data"};
  float col_positions[] = {0.0f, 0.15f, 0.30f, 0.45f, 0.60f, 0.75f, 0.90f};
  float col_widths[] = {0.15f, 0.15f, 0.15f, 0.15f, 0.15f, 0.15f, 0.10f};

  for (int i = 0; i < 7; ++i) {
    float x = bounds.x + bounds.width * col_positions[i];
    float width = bounds.width * col_widths[i];
    DrawText(headers[i], x + 5, bounds.y + 22, 12, (Color){64, 64, 64, 255}); // DARKGRAY in RGB
    DrawLine(bounds.x + x + width, bounds.y + 20, bounds.x + x + width, bounds.y + 20 + header_height, (Color){128, 128, 128, 255}); // GRAY in RGB
  }

  // Draw messages
  const auto& model_items = model_->getItems();
  float yPos = bounds.y + 40 + scroll_offset;
  int displayedCount = 0;
  const int maxDisplay = static_cast<int>((bounds.height - 45) / 20);

  for (size_t i = 0; i < model_items.size() && displayedCount < maxDisplay; ++i) {
    if (yPos > bounds.y + 40 && yPos < bounds.y + bounds.height) {
      drawRow(i, yPos, bounds, model_items[i]);
      yPos += 20;
      displayedCount++;
    } else if (yPos < bounds.y + 40) {
      yPos += 20;
    }
  }

  // Draw scroll indicator
  if (!model_items.empty()) {
    float scroll_ratio = -scroll_offset / (model_items.size() * 20.0f - bounds.height + 45);
    float scrollbar_height = bounds.height * 0.1f;
    if (scrollbar_height < 20.0f) scrollbar_height = 20.0f;
    if (scrollbar_height > bounds.height - 45) scrollbar_height = bounds.height - 45;

    float scrollbar_y = bounds.y + 40 + (bounds.height - 85) * scroll_ratio;

    DrawRectangle(bounds.x + bounds.width - 15, scrollbar_y, 10, scrollbar_height, (Color){128, 128, 128, 255}); // GRAY in RGB
  }
}

void MessagesWidget::drawRow(int index, float yPos, const Rectangle& bounds, const MessageListModel::Item& item) {
  Color textColor = (index == selected_message_index) ? (Color){0, 121, 241, 255} : (Color){64, 64, 64, 255}; // BLUE or DARKGRAY in RGB
  Color bgColor = (index == selected_message_index) ? Color{200, 200, 255, 255} : (Color){245, 245, 245, 255}; // RAYWHITE equivalent

  // Draw row background if selected
  if (index == selected_message_index) {
    DrawRectangle(bounds.x + 1, static_cast<int>(yPos - 2), static_cast<int>(bounds.width - 2), 18, bgColor);
  }

  // Draw row data
  std::string address_str = "0x" + std::to_string(item.id.address);
  std::string source_str = std::to_string(item.id.src);
  char freq_buf[20];
  snprintf(freq_buf, sizeof(freq_buf), "%.2fHz", item.frequency);
  std::string freq_str(freq_buf);
  std::string count_str = std::to_string(item.count);
  std::string data_str = item.data.empty() ? "00 00 00 00" : item.data;

  const std::string* values[] = {&item.name, &source_str, &address_str, &item.node, &freq_str, &count_str, &data_str};
  float col_positions[] = {0.0f, 0.15f, 0.30f, 0.45f, 0.60f, 0.75f, 0.90f};

  for (int i = 0; i < 7; ++i) {
    float x = bounds.x + bounds.width * col_positions[i] + 5;
    DrawText(values[i]->c_str(), static_cast<int>(x), static_cast<int>(yPos), 10, textColor);
  }
}

void MessagesWidget::handleInput() {
  if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
    Vector2 mousePos = GetMousePosition();
    if (CheckCollisionPointRec(mousePos, bounds_)) {
      // Calculate which message was clicked based on scroll position
      const auto& model_items = model_->getItems();
      float relativeY = mousePos.y - bounds_.y - 40; // Adjust for header
      int messageIndex = static_cast<int>((relativeY - scroll_offset) / 20); // Approximate message height

      if (messageIndex >= 0 && messageIndex < static_cast<int>(model_items.size())) {
        selected_message_index = messageIndex;
        if (onMsgSelectionChanged) {
          onMsgSelectionChanged(model_items[messageIndex].id);
        }
      }
    }
  }

  // Handle scrolling
  handleScrolling();
}

void MessagesWidget::handleScrolling() {
  const auto& model_items = model_->getItems();
  if (IsKeyDown(KEY_UP) || GetMouseWheelMove() > 0) {
    scroll_offset += 20;
    if (scroll_offset > 0) scroll_offset = 0;
  }
  if (IsKeyDown(KEY_DOWN) || GetMouseWheelMove() < 0) {
    scroll_offset -= 20;
    float maxScroll = -static_cast<int>(model_items.size()) * 20 + bounds_.height - 45;
    if (scroll_offset < maxScroll) scroll_offset = maxScroll;
  }

  // Mouse drag scrolling
  if (IsMouseButtonDown(MOUSE_LEFT_BUTTON)) {
    Vector2 current_mouse = GetMousePosition();
    if (CheckCollisionPointRec(current_mouse, bounds_) && !is_dragging) {
      if (current_mouse.y > bounds_.y + 40) { // Only in content area
        last_mouse_pos = current_mouse;
        is_dragging = true;
      }
    } else if (is_dragging) {
      float deltaY = current_mouse.y - last_mouse_pos.y;
      scroll_offset += deltaY;

      float maxScroll = -static_cast<int>(model_items.size()) * 20 + bounds_.height - 45;
      if (scroll_offset > 0) scroll_offset = 0;
      if (scroll_offset < maxScroll) scroll_offset = maxScroll;

      last_mouse_pos = current_mouse;
    }
  } else {
    is_dragging = false;
  }
}

void MessagesWidget::selectMessage(const MessageId &message_id) {
  const auto& model_items = model_->getItems();
  for (size_t i = 0; i < model_items.size(); ++i) {
    if (model_items[i].id == message_id) {
      selected_message_index = i;
      // Scroll to make the selected message visible
      float item_pos = i * 20 + scroll_offset;
      if (item_pos < bounds_.y) {
        scroll_offset = bounds_.y - i * 20;
      } else if (item_pos > bounds_.y + bounds_.height - 20) {
        scroll_offset = bounds_.y + bounds_.height - 20 - i * 20;
      }
      break;
    }
  }
}

void MessagesWidget::updateTitle() {
  // Update title based on current state
  title = "MESSAGES (" + std::to_string(model_->getItems().size()) + ")";
  if (onTitleChanged) {
    onTitleChanged(title);
  }
}

const std::vector<MessageListModel::Item>& MessagesWidget::getItems() const {
  return model_->getItems();
}

void MessagesWidget::setModel(std::unique_ptr<MessageListModel> model) {
  model_ = std::move(model);
  update();
}