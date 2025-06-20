#pragma once

#pragma once
#include <utility>

enum class PointType { Base, Place };

class Point {
public:
    std::pair<double, double> coordinates;
    QString name;

    Point() = default;
    Point(double lng, double lat) : coordinates{ lng, lat } {}

    virtual ~Point() = default;
    virtual PointType type() const { return PointType::Base; }
};