#pragma once


struct TilePixel {
    int tileX;
    int tileY;
    int pixelX;
    int pixelY;
};

struct LineSegment {
    TilePixel start;
    TilePixel end;
};