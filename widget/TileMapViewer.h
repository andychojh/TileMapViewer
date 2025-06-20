#pragma once

#include <QGraphicsView>
#include <QGraphicsScene>
#include <QWheelEvent>
#include <QKeyEvent>
#include <QPoint>
#include <QPixmap>
#include <QGraphicsPixmapItem>
#include <QMap>
#include <QFile>
#include <map>
#include <vector>
#include <Qmenu>
#include <QLineEdit>

using namespace std; 

class TileMapViewer : public QGraphicsView {
    Q_OBJECT

public:
    TileMapViewer(QWidget* parent = nullptr);  // 💡 Initialize the tile world
    int zoom_level;          // 🔍 Current zoom level (int, like 14 ~ 19)
    QGraphicsScene* scene;
    map<int, vector<pair<int, int>>> tilesMap;
    QPoint pos;              // 🧭 Current center tile coordinate (x, y)
    void loadTiles();        // 🔁 Load tiles into scene
    void setDeparture(const QString& departure) { departureBox->setText(departure); }
    void setDestination(const QString& destination) { destinationBox->setText(destination); }
    QLineEdit* departureBox = nullptr;
    QLineEdit* destinationBox = nullptr;
    vector<QGraphicsItem*> routeItems;
    bool routeHighlight = false;                // 경로 탐색시 활성화 되는 flag

protected:
    void wheelEvent(QWheelEvent* event) override;   // 🖱️ Mouse wheel = zoom
    void keyPressEvent(QKeyEvent* event) override;  // ⌨️ Arrow keys = move
    void zoomTransform(int zoom_level, bool zoom_in); 
    void mousePressEvent(QMouseEvent* event); 
    void mouseMoveEvent(QMouseEvent* event); 
    void mouseReleaseEvent(QMouseEvent* event);
  
private:
    QPoint lastMousePos;     // 마지막 마우스 위치 
    QPoint dragStartPos;     // 드래그 시작 때의 pos 저장
    bool dragging = false;    // 드래그 상태 플래그 

    double currentLat; 
    double currentLng; 

    const int tileSize = 256;                         // 📦 Tile pixel size
    const int minZoom = 14;
    const int maxZoom = 19;
    const QString tilePathTemplate = "Seocho_Tiles_Colored6/%1/%2/%3.png";  // 🗂️ Tile path format
    
    const int minX[7] = { 6985, 13971, 27942, 55885, 111771, 223542, 447085 };
    const int minY[7] = { 3173, 6347, 12695, 25391, 50783, 101566, 203132 };

    map<int, vector<pair<int, int>>> loadTilesCoordinatesByZoom(const QString& zoomLevelRootDir);
    std::pair<double, double> tilePixelToLonLat(int z, int x, int y, int pixel_x, int pixel_y);

   

    QMenu menu; 
};
