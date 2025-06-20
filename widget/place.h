#pragma once

#pragma once
#include "point.h"
#include <QString>

class Place : public Point {
public:
    QString address;
    QString amenity;

    Place() = default;

    Place(double lng, double lat, const QString& name = {},
        const QString& address = {}, const QString& amenity = {})
        : Point(lng, lat), address(address), amenity(amenity)
    {
        this->name = name;
    }

    QString label() const {
        return name.isEmpty() ? "[Unknown Place]" : name;
    }

    PointType type() const override { return PointType::Place; }
};
