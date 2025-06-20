#include "widget.h"
#include "Navigator.h"
#include <QtWidgets/QApplication>
#include <vector>
#include <map>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QDebug>
#include <cmath>
#include <queue>
#include <WalkNode.h>
#include <QIcon>

using namespace std; 

int main(int argc, char* argv[])
{
    QApplication a(argc, argv);
    widget w;
    qDebug() << "Debug Message";
    w.show(); 
    return a.exec();
}






