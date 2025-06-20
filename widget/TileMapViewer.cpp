#include "TileMapViewer.h"
#include <QDebug>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <vector>
#include <iostream>
#include <filesystem>
#include <limits>
#include <QtMath>
#include <QDir>


using namespace std;
const int CENTERX = 337;
const int CENTERY = 337;
const int TILESIZE = 256;
namespace fs = std::filesystem;
const double PI = 3.14159265358979323846;


// QGraphicsView위에서 픽셀 좌표를 (경도,위도)로 변환 
std::pair<double, double> TileMapViewer::tilePixelToLonLat(int z, int x, int y, int pixel_x, int pixel_y)
{
    int tile_size = 256;
    double n = std::pow(2.0, z);

    // 전체 픽셀 단위에서의 위치
    double world_pixel_x = x * tile_size + pixel_x;
    double world_pixel_y = y * tile_size + pixel_y;

    // 경도
    double lon_deg = world_pixel_x / (tile_size * n) * 360.0 - 180.0;

    // 위도(rad): 메르카토르 변환
    double lat_rad = std::atan(std::sinh(PI * (1 - 2.0 * world_pixel_y / (tile_size * n))));
    double lat_deg = lat_rad * 180.0 / PI;

    // 결과: {경도, 위도}
    return { lon_deg, lat_deg };
}

// Zoom level: tile (X,Y) 로 저장 
map<int, vector<pair<int, int>>> TileMapViewer::loadTilesCoordinatesByZoom(const QString& zoomLevelRootDir)
{
    map<int, vector<pair<int, int>>> tilesMap;

    QDir rootDir(zoomLevelRootDir);
    if (!rootDir.exists()) {
        qWarning() << "Root directory does not exist:" << zoomLevelRootDir;
        return tilesMap;
    }

    QStringList zoomDirs = rootDir.entryList(QDir::Dirs | QDir::NoDotAndDotDot);
    for (const QString& zoomStr : zoomDirs) {
        bool okZ = false;
        int zoom_level = zoomStr.toInt(&okZ);
        if (!okZ) continue;

        QDir zoomDir(rootDir.filePath(zoomStr));
        QStringList xDirs = zoomDir.entryList(QDir::Dirs | QDir::NoDotAndDotDot);
        for (const QString& xStr : xDirs) {
            bool okX = false;
            int x = xStr.toInt(&okX);
            if (!okX) continue;

            QDir xDir(zoomDir.filePath(xStr));
            QStringList yFiles = xDir.entryList(QStringList() << "*.png", QDir::Files);
            for (const QString& yFile : yFiles) {
                QString yStr = yFile;
                if (yStr.endsWith(".png"))
                    yStr.chop(4);
                bool okY = false;
                int y = yStr.toInt(&okY);
                if (!okY) continue;

                tilesMap[zoom_level].push_back(QPair<int, int>(x, y));
            }
        }
    }
    return tilesMap;
}

// TileMapViewer Constructor, pos는 최상위 zoom_level에서 중앙 타일로 초기화 
TileMapViewer::TileMapViewer(QWidget* parent)
    : QGraphicsView(parent), scene(new QGraphicsScene(this)), pos(13972, 6348), zoom_level(14) {
    setScene(scene);
    setRenderHint(QPainter::Antialiasing);
    setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
    setDragMode(QGraphicsView::ScrollHandDrag);
    loadTiles();
    centerOn(0, 0);
    tilesMap = loadTilesCoordinatesByZoom("Seocho_Tiles_Colored6");
}


void TileMapViewer::zoomTransform(int zoom_level, bool zoom_in)
{
    if (zoom_in) {
        if (zoom_level == 14 || zoom_level == 17)
        {
            pos.setX(pos.x() * 2);
            pos.setY(pos.y() * 2); 
        }
        else {
            pos.setX(pos.x() * 2+1);
            pos.setY(pos.y() * 2+1);
        }
    }
    else {
        if (zoom_level == 15 || zoom_level == 18) {
            pos.setX(pos.x() / 2); 
            pos.setY(pos.y() / 2);
        }
        else {
            pos.setX((pos.x() - 1) / 2); 
            pos.setY((pos.y() - 1) / 2); 
        }
    }
}

void TileMapViewer::wheelEvent(QWheelEvent* event) {
    int delta = event->angleDelta().y();

    QPoint mousePos = event->position().toPoint();
    int xTile = (mousePos.x() - CENTERX) / tileSize;
    int yTile = (mousePos.y() - CENTERY) / tileSize;
    pos.setX(pos.x() + xTile); 
    pos.setY(pos.y() + yTile); 

    if (delta > 0 && zoom_level < maxZoom)
    {
        zoomTransform(zoom_level, true); 
        zoom_level++;

    }
    else if (delta < 0 && zoom_level > minZoom) {
        zoomTransform(zoom_level, false); 
        zoom_level--;
    }

    loadTiles();
}

void TileMapViewer::keyPressEvent(QKeyEvent* event) {
    switch (event->key()) {
    case Qt::Key_Left:  pos.setX(pos.x() - 1); break;
    case Qt::Key_Right: pos.setX(pos.x() + 1); break;
    case Qt::Key_Up:    pos.setY(pos.y() - 1); break;
    case Qt::Key_Down:  pos.setY(pos.y() + 1); break;
    }
    loadTiles();
}

// 1. 마우스 버튼 누름: 드래그 시작 설정
void TileMapViewer::mousePressEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton) {
        dragging = true;
        lastMousePos = event->pos();
        dragStartPos = pos;  // 현재 맵 좌표 기억
    }
    else if(event->button() == Qt::RightButton) {

       // Complete the Code

        QMenu menu(this);  // 매번 새로운 메뉴 객체 생성

        // 현재 Pixel 좌표를 tilePixelToLonLat 을 통해 (경도, 위도) 로 변환
        QPoint mousePos = event->pos();
        int tileXOffset = mousePos.x() / tileSize;
        int tileYOffset = mousePos.y() / tileSize;

        int pixXOffset = mousePos.x() % tileSize;
        int pixYOffset = mousePos.y() % tileSize;

        int tileXindex = pos.x() + tileXOffset - 1;
        int tileYindex = pos.y() + tileYOffset - 1;

        pair<double, double> coord = tilePixelToLonLat(zoom_level, tileXindex, tileYindex,
            pixXOffset, pixYOffset);

        currentLat = coord.first;
        currentLng = coord.second;

        // 메뉴와 메뉴 액션들 생성
        QMenu contextMenu(this);
        contextMenu.addAction(QString("경도: %1").arg(currentLng, 0, 'f', 7))->setEnabled(false);
        contextMenu.addAction(QString("위도: %1").arg(currentLat, 0, 'f', 7))->setEnabled(false);
        contextMenu.addSeparator();

        // 출발지/목적지 설정 메뉴
        QAction* departureAction = contextMenu.addAction("출발지 설정");
        QAction* destinationAction = contextMenu.addAction("목적지 설정");
        QAction* selectedAction = contextMenu.exec(event->globalPosition().toPoint()); // 액션 선택

        if (selectedAction) { // 메뉴 안을 눌렀으면
            if (selectedAction == departureAction) {
                QString coordText = QString::number(currentLng, 'f', 7) + ", " + QString::number(currentLat, 'f', 7);
                this->setDeparture(coordText);
            }
            else if (selectedAction == destinationAction) {
                QString coordText = QString::number(currentLng, 'f', 7) + ", " + QString::number(currentLat, 'f', 7);
                this->setDestination(coordText);
            }
        }
    }
}

// 2. 마우스 이동: pos 갱신
void TileMapViewer::mouseMoveEvent(QMouseEvent* event) {
    QPointF panOffset;
    if (dragging) {
        QPoint delta = event->pos() - lastMousePos;  // 마우스 이동량(픽셀)
        // 한 타일 사이즈씩 넘겼을 때만 pos 업데이트
        int dx = -delta.x() / tileSize; // ← 오른쪽방향일수록 pos.x()는 감소
        int dy = -delta.y() / tileSize; // ↑ 아래일수록 pos.y()는 감소
        // (움직인 픽셀만큼 좌표(타일단위) 이동)
        pos = dragStartPos + QPoint(dx, dy);
        loadTiles();
    }
}

// 3. 마우스 버튼 뗌: 드래그 종료
void TileMapViewer::mouseReleaseEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton) {
        dragging = false;
    }
}


void TileMapViewer::loadTiles() {
    
    scene->clear(); 
    int range = 1;  // Load 3x3 tiles centered around pos
    for (int dx = -range; dx <= range; ++dx) {
        for (int dy = -range; dy <= range; ++dy) {
            int x = pos.x() + dx;
            int y = pos.y() + dy;
            QString path = tilePathTemplate.arg(zoom_level).arg(x).arg(y);
            if (QFile::exists(path)) {
              // Complete the Code 
                QPixmap tile(path);
                QGraphicsPixmapItem* item = scene->addPixmap(tile);
                item->setPos(dx * tileSize - tileSize / 2, dy * tileSize - tileSize / 2);
            }
            else {
                QPixmap tile(tileSize, tileSize);
                tile.fill(Qt::white);
                QGraphicsPixmapItem* item = scene->addPixmap(tile);
                item->setPos((dx+1) * tileSize - tileSize/2, (dy+1) * tileSize - tileSize/2);
            }
        }
    }
}
