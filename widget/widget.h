#pragma once

#include <QtWidgets/QMainWindow>
#include "TIleMapViewer.h"
#include "Navigator.h"
#include "ui_widget.h"
#include <vector>
#include "WalkNode.h"
#include "point.h"
#include "place.h"
#include "TileTypes.h"
#include <QIcon> 
#include <QPixmap>


struct queryItem {
    const Point* point;
    int matchIndex;

    bool operator<(const queryItem& other) const {
        if (matchIndex != other.matchIndex)
            return matchIndex < other.matchIndex;
        return point->name.length() < other.point->name.length();
    }
};

class Navigator; 

class widget : public QMainWindow
{
    Q_OBJECT

public:
    widget(QWidget *parent = nullptr);
    ~widget();
    void routeUpdate(const vector<int> path,
        const vector<WalkNode> routeNodes,
        const vector<WayTrack> wayTracks);

    vector<Point*> points;
    vector<WalkNode> routeNodes;        
    vector<WayTrack> wayTracks;
    vector<int> tpOffsets;

public slots:
    void routeClicked();  
    void routeInfoDisplay(const vector<WayTrack> wayTracks);
    void searchPlace(QString query);
    void placeClicked(QListWidgetItem* place); 
    void onDepartureSet(double lat, double lng);
    void onDestinationSet(double lat, double lng);
    void routeRender();

private:
    Ui::widgetClass ui;
    TileMapViewer *tilemapViewer; 
    Navigator *navigator; 
    QIcon turnLeftIcon;
    QIcon straightIcon; 
    QIcon turnRightIcon;
    
    // route를 렌더링 하는데 필요한 painting tool 
    const int pointRadius = 8; 
    QPen routePen; 
    QPen highlightPen;
    QPen tpPen; 
    QBrush tpBrush;

    QPixmap markerStart;
    QPixmap markerDest;
    vector<const Point*> queriedReturns;    
    void widgetStyleSetup();
    TilePixel latLontoTilePixel(double lat, double lon, int zoom_level);
    
};
