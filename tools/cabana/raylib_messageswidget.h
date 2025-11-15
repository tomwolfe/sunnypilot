#pragma once

#include <vector>
#include <string>
#include <memory>
#include <set>

#include "tools/cabana/dbc/dbcmanager.h" // This includes cereal/messaging/messaging.h
#include "tools/cabana/streams/abstractstream.h"
// Define OPENPILOT_RAYLIB before including raylib to prevent enum conflicts
#define OPENPILOT_RAYLIB
#include "third_party/raylib/include/raylib.h"

// Message list model for Raylib-based UI
class MessageListModel {
public:
  struct Item {
    MessageId id;
    std::string name;
    std::string node;
    bool operator==(const Item &other) const {
      return id == other.id && name == other.name && node == other.node;
    }
  };
  
  MessageListModel();
  void sort(int column, bool ascending = true);
  void setFilterStrings(const std::map<int, std::string> &filters);
  void showInactiveMessages(bool show);
  void msgsReceived(const std::set<MessageId> *new_msgs, bool has_new_ids);
  bool filterAndSort();
  void dbcModified();
  
  const std::vector<Item>& getItems() const { return filtered_items_; }
  void setItems(const std::vector<Item>& newItems) { items_ = newItems; filterAndSort(); }
  
  enum Column {
    NAME = 0,
    SOURCE,
    ADDRESS,
    NODE,
    FREQ,
    COUNT,
    DATA,
  };

private:
  void sortItems(std::vector<Item> &items);
  bool match(const Item &item);

  std::vector<Item> items_;
  bool show_inactive_messages = true;
  
  std::vector<Item> filtered_items_;
  std::map<int, std::string> filters_;
  std::set<MessageId> dbc_messages_;
  int sort_column = 0;
  bool sort_order = true; // true for ascending, false for descending
};

// Raylib-based MessagesWidget
class MessagesWidget {
public:
  MessagesWidget(void* parent = nullptr);  // parent is just for API compatibility
  ~MessagesWidget();
  
  void update();
  void render(const Rectangle& bounds);
  void handleInput();
  
  void selectMessage(const MessageId &message_id);
  void updateTitle();
  
  // Accessor methods for UI state
  const std::vector<MessageListModel::Item>& getItems() const;
  void setModel(std::unique_ptr<MessageListModel> model);
  
  // Signals equivalent - using callback functions
  std::function<void(const MessageId&)> onMsgSelectionChanged;
  std::function<void(const std::string&)> onTitleChanged;

private:
  void drawRow(int index, float yPos, const Rectangle& bounds, const MessageListModel::Item& item);
  void handleScrolling();
  
  std::unique_ptr<MessageListModel> model_;
  int selected_message_index = -1;
  float scroll_offset = 0.0f;
  std::string title = "MESSAGES";
  
  Rectangle bounds_;
  bool is_visible = true;
  
  // Input state
  Vector2 last_mouse_pos = {0, 0};
  bool is_dragging = false;
};