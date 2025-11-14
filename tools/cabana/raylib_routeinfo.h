#pragma once

#include <vector>
#include <string>

#include "raylib.h"
#include "tools/cabana/streams/replaystream.h"

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