// Copyright 2020 Varjo Technologies Oy. All rights reserved.
#ifndef VARJO_TYPES_EXPERIMENTAL_H
#define VARJO_TYPES_EXPERIMENTAL_H

#include "Varjo_types.h"
#include "Varjo_types_layers.h"
#include "Varjo_types_layers_experimental.h"

#if defined __cplusplus
extern "C" {
#endif

struct varjo_Quaternion {
    double x;  //!< X coordinate.
    double y;  //!< Y coordinate.
    double z;  //!< Z coordinate.
    double w;  //!< W
};

typedef int64_t varjo_EyeVisibility;
static const varjo_EyeVisibility varjo_EyeVisibilityBoth = 0;
static const varjo_EyeVisibility varjo_EyeVisibilityLeft = 1;
static const varjo_EyeVisibility varjo_EyeVisibilityRight = 2;

static const varjo_LayerType varjo_LayerQuadType = 0x2;

struct varjo_LayerQuad {
    struct varjo_LayerHeader header;
    varjo_Space space;
    struct varjo_SwapChainViewport viewport;
    varjo_EyeVisibility eyeVisibility;
    struct varjo_Vector3D position;
    struct varjo_Quaternion orientation;
    double width;
    double height;
};

#if defined __cplusplus
}
#endif

#endif  // VARJO_TYPES_EXPERIMENTAL_H