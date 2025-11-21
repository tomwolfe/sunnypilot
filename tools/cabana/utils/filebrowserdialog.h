#pragma once

#include <string>
#include <vector>
#include <functional>

#define OPENPILOT_RAYLIB
#include "third_party/raylib/include/raylib.h"

// Represents a file or directory in the browser
struct FileEntry {
    std::string name;
    bool is_directory;
    std::string path;
    
    FileEntry(const std::string& n, bool dir, const std::string& p) 
        : name(n), is_directory(dir), path(p) {}
};

// File browser dialog for Raylib-based UI
class FileBrowserDialog {
public:
    FileBrowserDialog();
    ~FileBrowserDialog() = default;

    // Show the dialog and return the selected file path
    std::string browseFile(const std::string& title = "Select File", 
                          const std::string& initial_path = "",
                          const std::string& filter = "");
    
    std::string browseDirectory(const std::string& title = "Select Directory",
                               const std::string& initial_path = "");
    
    void show();
    void hide();
    void update();
    void render();

    // Callback for when a file/directory is selected
    std::function<void(const std::string&)> onFileSelected;
    std::function<void()> onCancel;

    bool isVisible() const { return visible_; }

private:
    void loadDirectory(const std::string& path);
    void updateFileEntries();
    void handleInput();
    void drawFileList();
    void drawPathBar();
    void drawButtons();
    
    std::vector<FileEntry> file_entries_;
    std::string current_path_;
    std::string title_;
    std::string filter_;
    std::string selected_path_;
    
    bool visible_ = false;
    bool is_directory_mode_ = false;
    
    // UI bounds
    Rectangle window_bounds_;
    Rectangle path_bar_bounds_;
    Rectangle file_list_bounds_;
    Rectangle buttons_bounds_;
    Rectangle ok_button_bounds_;
    Rectangle cancel_button_bounds_;
    
    // UI state
    int selected_index_ = -1;
    float scroll_offset_ = 0.0f;
    
    // Input state
    Vector2 last_mouse_pos_ = {0, 0};
    bool is_dragging_ = false;
};

// Global file browser instance
extern FileBrowserDialog* fileBrowserDialog;