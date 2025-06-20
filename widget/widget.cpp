#include "widget.h"
#include <QtMath>
#include <vector>
#include <iostream>
#include <QPixMap>
#include <QPen>
#include <Qcolor>
#include <QScrollBar>
#include <algorithm>

const int CENTERX = 337;
const int CENTERY = 337;
const int TILESIZE = 256; 


TilePixel widget::latLontoTilePixel(double lat, double lon, int zoom_level) {
    const double latRad = lat * M_PI / 180.0;
    int n = pow(2, zoom_level);

    double x = (lon + 180.0) / 360.0;
    double y = (1.0 - (log(tan(latRad) + 1.0 / cos(latRad)) / M_PI)) / 2.0;  // 북위 +85.0511 -> y=0, 남위 -85.0511 -> y=1   

    int tileX = (int)floor(x * n);
    int tileY = (int)floor(y * n);

    int pixelX = (int)(floor((x * n - tileX) * TILESIZE));
    int pixelY = (int)(floor((y * n - tileY) * TILESIZE));

    return TilePixel{ tileX, tileY, pixelX, pixelY };
}

widget::widget(QWidget *parent)
    : QMainWindow(parent)
    , markerStart("./icons/departure.png")
    , markerDest("./icons/destination.png")

{
    ui.setupUi(this);
    tilemapViewer = new TileMapViewer(this);
    ui.horizontalLayout->addWidget(tilemapViewer);
    
    // Painting tools 설정 
    routePen.setColor(Qt::red);
    routePen.setWidth(5);
    highlightPen.setColor(Qt::green); 
    highlightPen.setWidth(5); 
    tpPen.setColor(Qt::darkGreen);
    tpPen.setWidth(1);
    tpBrush.setColor(Qt::green);

    // 방향 icon setup 
    turnLeftIcon = QIcon("./icons/turn-left.png");
    straightIcon = QIcon("./icons/straight.png"); 
    turnRightIcon = QIcon("./icons/turn-right.png");

    navigator = new Navigator(); 
    navigator->run(); 

    // 컴포넌트 연결
    connect(ui.routeButton, SIGNAL(clicked()), this, SLOT(routeClicked()));
    connect(ui.searchBox, SIGNAL(textChanged(const QString&)), this, SLOT(searchPlace(const QString&)));
    connect(ui.listWidget, SIGNAL(itemDoubleClicked(QListWidgetItem*)), this, SLOT(placeClicked(QListWidgetItem*)));
    
    widgetStyleSetup();
}


void widget::onDepartureSet(double lat, double lng)
{

}

void widget::onDestinationSet(double lat, double lng)
{

}

pair<int, int> findClosestTile(const vector<pair<int, int>>& tileXYs, const pair<double, double>& location) {
    if (tileXYs.empty()) {
        return { -1, -1 }; // 빈 경우 표시용
    }

    pair<int, int> closest = tileXYs[0];
    double minDist = std::numeric_limits<double>::max();

    for (const auto& tileXY : tileXYs) {
        double dx = tileXY.first - location.first;  // int - double -> double
        double dy = tileXY.second - location.second;
        double dist = std::sqrt(dx * dx + dy * dy);

        if (dist < minDist) {
            minDist = dist;
            closest = tileXY;
        }
    }

    return closest;
}


void widget::routeInfoDisplay(const vector<WayTrack> wayTracks) {

    QIcon dirIcon;
    QString dirMsg;

}

void widget::routeRender(){
    
}


void widget::routeClicked() {    
    routeRender(); 
    routeInfoDisplay(wayTracks); // 경로 정보 listwidget에 표시 
}


void widget::searchPlace(QString query)
{


}

void widget::placeClicked(QListWidgetItem* place)
{
  

}

void widget::widgetStyleSetup()
{
    /*** Style Setup***/
    QIcon icon("./icons/appIcon.jpg");
    QPixmap bkgnd("wallpaper/wallpaper.png");
    bkgnd = bkgnd.scaled(this->size(), Qt::IgnoreAspectRatio);
    QPalette palette;
    palette.setBrush(QPalette::Window, bkgnd);
    this->setPalette(palette);
    this->setWindowIcon(icon);

    ui.searchBox->setStyleSheet(
        "background: rgba(255, 255, 255, 40%);"  // 살짝 투명
        "border: 1px solid #C0A060;"              // 연한 금색 테두리
        "border-radius: 8px;"
        "color: white;"
        "font-family: 'Josefin Sans';"
        "padding: 5px;"
    );
    ui.departure->setStyleSheet(
        "background: rgba(255, 255, 255, 20%);"
        "border: 1px solid #C0A060;"
        "color: white;"
        "font-family: 'Josefin Sans';"
    );
    ui.destination->setStyleSheet(
        "background: rgba(255, 255, 255, 20%);"
        "border: 1px solid #C0A060;"
        "color: white;"
        "font-family: 'Josefin Sans';"
    );
    ui.listWidget->setStyleSheet(
        "QListWidget {"
        "  background-color: rgba(10, 47, 58, 180);" // 어두운 청록, 약간 투명
        "  color: #EAD8B4;"                          // 따뜻한 크림/골드톤 텍스트
        "  border: 1px solid #C0A060;"               // 연한 골드 테두리
        "  font-family: 'Josefin Sans';"
        "  font-size: 14px;"
        "}"

        "QListWidget::item:selected {"
        "  background-color: rgba(255, 215, 0, 60);" // 연한 금색 빛 배경 (선택 항목)
        "  color: white;"                             // 선택됐을 때 글자색
        "  border: 1px solid #FFD700;"                // 진짜 금색 테두리
        "}"

        "QScrollBar:vertical {"
        "  background: transparent;"
        "  width: 8px;"
        "  margin: 2px 0 2px 0;"
        "}"

        "QScrollBar::handle:vertical {"
        "  background: #C0A060;"                      // 스크롤 핸들 골드색
        "  min-height: 20px;"
        "  border-radius: 4px;"
        "}"

        "QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {"
        "  height: 0px;"
        "}"

        "QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {"
        "  background: none;"
        "}"
    );
    
    QString scrollStyle = R"(
    QScrollBar:vertical {
        background: transparent;
        width: 12px;
        margin: 0px;
    }
    QScrollBar::handle:vertical {
        background: #C0A060;
        min-height: 20px;
        border-radius: 6px;
    }
    QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
        height: 0px;
    }
    QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {
        background: none;
    }

    QScrollBar:horizontal {
        background: transparent;
        height: 12px;
        margin: 0px;
    }
    QScrollBar::handle:horizontal {
        background: #C0A060;
        min-width: 20px;
        border-radius: 6px;
    }
    QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {
        width: 0px;
    }
    QScrollBar::add-page:horizontal, QScrollBar::sub-page:horizontal {
        background: none;
    }
    )";
    ui.routeButton->setStyleSheet(R"(
    QPushButton {
    color: #4B3F2F;
    font-weight: bold;
    border: 1px solid #D6C4A2;
    border-radius: 6px;
    padding: 6px 12px;
    background: qlineargradient(
        x1:0, y1:0, x2:0, y2:1,
        stop:0   #F5E8D0,
        stop:0.5 #EAD8B4,
        stop:1   #DCC9A3
    );
    }

    QPushButton:hover {
        background: qlineargradient(
            x1:0, y1:0, x2:0, y2:1,
            stop:0   #FFF5DD,
            stop:1   #EFDDBF
        );
    }

    QPushButton:pressed {
        background: qlineargradient(
            x1:0, y1:0, x2:0, y2:1,
            stop:0   #DCC9A3,
            stop:1   #C8B38F
        );
    }
    )");
    tilemapViewer->verticalScrollBar()->setStyleSheet(scrollStyle);
    tilemapViewer->horizontalScrollBar()->setStyleSheet(scrollStyle);
  
    ui.searchBox->setClearButtonEnabled(true);
    ui.searchBox->addAction(QIcon("icons/search.svg"), QLineEdit::LeadingPosition);
    ui.searchBox->setPlaceholderText("Search...");
    
    ui.departure->setPlaceholderText("출발지"); 
    ui.destination->setPlaceholderText("목적지"); 
}


// walkNodes, wayTracks 업데이트 
void widget::routeUpdate(const vector<int> path, const vector<WalkNode> walkNodes, const vector<WayTrack> Tracks)
{

    for (size_t i = 0; i < path.size(); ++i) {
        routeNodes.push_back(walkNodes[path[i]]);
    }
    wayTracks = Tracks;

}

widget::~widget()
{
    
}
