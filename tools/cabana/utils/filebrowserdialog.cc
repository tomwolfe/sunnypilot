#include "tools/cabana/utils/filebrowserdialog.h"
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>
#include <algorithm>
#include <sstream>
#include <cstring>

// Global instance
FileBrowserDialog* fileBrowserDialog = nullptr;

FileBrowserDialog::FileBrowserDialog() {
    // Initialize default window bounds
    window_bounds_ = {200, 100, 600, 500};
    path_bar_bounds_ = {window_bounds_.x + 20, window_bounds_.y + 40, 560, 30};
    file_list_bounds_ = {window_bounds_.x + 20, window_bounds_.y + 80, 560, 340};
    buttons_bounds_ = {window_bounds_.x + 20, window_bounds_.y + 440, 560, 40};
    ok_button_bounds_ = {window_bounds_.x + 300, window_bounds_.y + 445, 120, 30};
    cancel_button_bounds_ = {window_bounds_.x + 440, window_bounds_.y + 445, 120, 30};
    
    // Get initial directory
    char current_dir[1024];
    if (getcwd(current_dir, sizeof(current_dir))) {
        current_path_ = std::string(current_dir);
    } else {
        current_path_ = "/";
    }
}

std::string FileBrowserDialog::browseFile(const std::string& title, 
                                        const std::string& initial_path, 
                                        const std::string& filter) {
    title_ = title;
    filter_ = filter;
    is_directory_mode_ = false;
    
    if (!initial_path.empty()) {
        current_path_ = initial_path;
    }
    
    show();
    return selected_path_;
}

std::string FileBrowserDialog::browseDirectory(const std::string& title,
                                             const std::string& initial_path) {
    title_ = title;
    is_directory_mode_ = true;
    
    if (!initial_path.empty()) {
        current_path_ = initial_path;
    }
    
    show();
    return selected_path_;
}

void FileBrowserDialog::show() {
    loadDirectory(current_path_);
    visible_ = true;
}

void FileBrowserDialog::hide() {
    visible_ = false;
    selected_path_.clear();
}

void FileBrowserDialog::update() {
    if (!visible_) return;
    
    handleInput();
    updateFileEntries();
}

void FileBrowserDialog::handleInput() {
    Vector2 mouse_pos = GetMousePosition();
    
    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
        // Check if clicked in file list
        if (CheckCollisionPointRec(mouse_pos, file_list_bounds_)) {
            // Calculate which file was clicked
            float relative_y = mouse_pos.y - file_list_bounds_.y - 25; // Offset for header
            int row_height = 25;
            int clicked_row = static_cast<int>(relative_y / row_height) + static_cast<int>(scroll_offset_ / row_height);
            
            if (clicked_row >= 0 && clicked_row < static_cast<int>(file_entries_.size())) {
                selected_index_ = clicked_row;
                const auto& entry = file_entries_[clicked_row];
                
                if (entry.is_directory) {
                    // Navigate to directory
                    loadDirectory(entry.path);
                    selected_index_ = -1; // Reset selection after directory change
                } else if (!is_directory_mode_) {
                    // Select file if in file mode
                    selected_path_ = entry.path;
                    if (onFileSelected) {
                        onFileSelected(selected_path_);
                    }
                }
            }
        }
        // Check OK button
        else if (CheckCollisionPointRec(mouse_pos, ok_button_bounds_)) {
            if (selected_index_ >= 0 && selected_index_ < static_cast<int>(file_entries_.size())) {
                const auto& entry = file_entries_[selected_index_];
                if (!entry.is_directory || is_directory_mode_) {
                    selected_path_ = entry.path;
                    hide();
                    if (onFileSelected) {
                        onFileSelected(selected_path_);
                    }
                }
            } else if (is_directory_mode_) {
                // If in directory mode and no file is selected, use current directory
                selected_path_ = current_path_;
                hide();
                if (onFileSelected) {
                    onFileSelected(selected_path_);
                }
            }
        }
        // Check Cancel button
        else if (CheckCollisionPointRec(mouse_pos, cancel_button_bounds_)) {
            hide();
            if (onCancel) {
                onCancel();
            }
        }
    }
    
    // Handle mouse wheel scrolling
    float wheel = GetMouseWheelMove();
    if (wheel != 0) {
        scroll_offset_ += wheel * 25 * 2; // Scroll speed
        scroll_offset_ = fmaxf(scroll_offset_, file_list_bounds_.height - file_entries_.size() * 25);
        scroll_offset_ = fminf(scroll_offset_, 0);
    }
}

void FileBrowserDialog::updateFileEntries() {
    // Update scroll position and other UI elements
    // Keep scroll bounds reasonable
    if (file_entries_.size() * 25 <= file_list_bounds_.height) {
        scroll_offset_ = 0;
    } else {
        float max_scroll = file_list_bounds_.height - file_entries_.size() * 25;
        if (scroll_offset_ > 0) scroll_offset_ = 0;
        if (scroll_offset_ < max_scroll) scroll_offset_ = max_scroll;
    }
}

void FileBrowserDialog::loadDirectory(const std::string& path) {
    current_path_ = path;
    file_entries_.clear();
    
    DIR* dir = opendir(path.c_str());
    if (!dir) return;
    
    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        std::string name = entry->d_name;
        
        // Skip . and .. entries
        if (name == "." || name == "..") continue;
        
        std::string full_path = path + "/" + name;
        struct stat statbuf;
        
        if (stat(full_path.c_str(), &statbuf) == 0) {
            bool is_dir = S_ISDIR(statbuf.st_mode);
            
            // If in file mode, filter by file extension if provided
            if (!is_directory_mode_ && !is_dir && !filter_.empty()) {
                size_t pos = name.rfind('.');
                if (pos != std::string::npos) {
                    std::string ext = name.substr(pos);
                    if (ext != filter_) continue; // Skip files that don't match filter
                }
            }
            
            file_entries_.emplace_back(name, is_dir, full_path);
        }
    }
    
    closedir(dir);
    
    // Sort: directories first, then files, both alphabetically
    std::sort(file_entries_.begin(), file_entries_.end(), 
              [](const FileEntry& a, const FileEntry& b) {
                  if (a.is_directory && !b.is_directory) return true;
                  if (!a.is_directory && b.is_directory) return false;
                  return a.name < b.name;
              });
    
    scroll_offset_ = 0;
}

void FileBrowserDialog::render() {
    if (!visible_) return;
    
    // Draw modal background
    DrawRectangle(0, 0, GetScreenWidth(), GetScreenHeight(), Fade((Color){0, 0, 0, 255}, 0.5f));
    
    // Draw dialog window
    DrawRectangleRec(window_bounds_, (Color){245, 245, 245, 255});
    DrawRectangleLines(window_bounds_.x, window_bounds_.y, window_bounds_.width, window_bounds_.height, (Color){0, 0, 0, 255});
    
    // Draw title
    DrawText(title_.c_str(), window_bounds_.x + 10, window_bounds_.y + 10, 16, BLACK);
    
    // Draw path bar
    drawPathBar();
    
    // Draw file list
    drawFileList();
    
    // Draw buttons
    drawButtons();
}

void FileBrowserDialog::drawPathBar() {
    // Draw current path
    DrawText("Path:", path_bar_bounds_.x, path_bar_bounds_.y - 20, 12, BLACK);
    DrawRectangleRec(path_bar_bounds_, WHITE);
    DrawRectangleLines(path_bar_bounds_.x, path_bar_bounds_.y, path_bar_bounds_.width, path_bar_bounds_.height, BLACK);
    DrawText(current_path_.c_str(), path_bar_bounds_.x + 5, path_bar_bounds_.y + 5, 10, BLACK);
}

void FileBrowserDialog::drawFileList() {
    // Draw file list background
    DrawRectangleRec(file_list_bounds_, WHITE);
    DrawRectangleLines(file_list_bounds_.x, file_list_bounds_.y, file_list_bounds_.width, file_list_bounds_.height, BLACK);
    
    // Draw column headers
    DrawText("Name", file_list_bounds_.x + 10, file_list_bounds_.y + 5, 12, BLACK);
    DrawText("Type", file_list_bounds_.x + 300, file_list_bounds_.y + 5, 12, BLACK);
    
    // Draw separator line
    DrawLine(file_list_bounds_.x, file_list_bounds_.y + 20, file_list_bounds_.x + file_list_bounds_.width, file_list_bounds_.y + 20, BLACK);
    
    // Draw file entries
    float row_y = file_list_bounds_.y + 25 + scroll_offset_;
    int displayed_count = 0;
    const int max_display = static_cast<int>(file_list_bounds_.height / 25);
    
    for (size_t i = 0; i < file_entries_.size() && displayed_count < max_display; ++i) {
        if (row_y >= file_list_bounds_.y && row_y <= file_list_bounds_.y + file_list_bounds_.height) {
            // Highlight selected row
            if (static_cast<int>(i) == selected_index_) {
                DrawRectangle(file_list_bounds_.x + 1, static_cast<int>(row_y), 
                             file_list_bounds_.width - 2, 24, (Color){173, 216, 230, 255}); // Light blue
            }
            
            const auto& entry = file_entries_[i];
            
            // Draw name
            if (entry.is_directory) {
                // Draw folder icon
                DrawRectangle(file_list_bounds_.x + 15, static_cast<int>(row_y + 6), 8, 8, (Color){139, 69, 19, 255}); // Brown
                DrawRectangle(file_list_bounds_.x + 10, static_cast<int>(row_y + 2), 18, 12, (Color){139, 69, 19, 255}); // Brown
                DrawText(entry.name.c_str(), file_list_bounds_.x + 35, static_cast<int>(row_y + 3), 10, BLACK);
            } else {
                DrawText(entry.name.c_str(), file_list_bounds_.x + 15, static_cast<int>(row_y + 3), 10, BLACK);
            }
            
            // Draw type
            DrawText(entry.is_directory ? "Folder" : "File", file_list_bounds_.x + 300, static_cast<int>(row_y + 3), 10, BLACK);
            
            displayed_count++;
        }
        row_y += 25;
    }
    
    // Draw scroll indicator if needed
    if (file_entries_.size() * 25 > file_list_bounds_.height && !file_entries_.empty()) {
        float scrollbar_height = (file_list_bounds_.height * file_list_bounds_.height) / (file_entries_.size() * 25);
        if (scrollbar_height < 20) scrollbar_height = 20;
        
        float scrollbar_pos = (-scroll_offset_ / (file_entries_.size() * 25 - file_list_bounds_.height)) 
                             * (file_list_bounds_.height - scrollbar_height);
        if (scrollbar_pos < 0) scrollbar_pos = 0;
        if (scrollbar_pos > file_list_bounds_.height - scrollbar_height) {
            scrollbar_pos = file_list_bounds_.height - scrollbar_height;
        }
        
        DrawRectangle(file_list_bounds_.x + file_list_bounds_.width - 15, 
                     file_list_bounds_.y + scrollbar_pos, 10, scrollbar_height, GRAY);
    }
}

void FileBrowserDialog::drawButtons() {
    // Draw OK button
    Color ok_color = LIGHTGRAY;
    if (CheckCollisionPointRec(GetMousePosition(), ok_button_bounds_)) {
        ok_color = GRAY;
    }
    DrawRectangleRec(ok_button_bounds_, ok_color);
    DrawRectangleLines(ok_button_bounds_.x, ok_button_bounds_.y, 
                      ok_button_bounds_.width, ok_button_bounds_.height, BLACK);
    DrawText("OK", ok_button_bounds_.x + ok_button_bounds_.width/2 - 15, 
             ok_button_bounds_.y + 8, 16, BLACK);
    
    // Draw Cancel button
    Color cancel_color = LIGHTGRAY;
    if (CheckCollisionPointRec(GetMousePosition(), cancel_button_bounds_)) {
        cancel_color = GRAY;
    }
    DrawRectangleRec(cancel_button_bounds_, cancel_color);
    DrawRectangleLines(cancel_button_bounds_.x, cancel_button_bounds_.y, 
                      cancel_button_bounds_.width, cancel_button_bounds_.height, BLACK);
    DrawText("Cancel", cancel_button_bounds_.x + cancel_button_bounds_.width/2 - 25, 
             cancel_button_bounds_.y + 8, 16, BLACK);
}