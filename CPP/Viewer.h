//
// Created by Zikai Liu on 11/15/22.
//

#ifndef HOLOSCANNER_VIEWER_H
#define HOLOSCANNER_VIEWER_H

#include "PCDDataTypes.h"

#include <igl/read_triangle_mesh.h>
#include <igl/readTGF.h>
#include <igl/readDMAT.h>
#include <igl/opengl/glfw/Viewer.h>

#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <igl/slice.h>

class Viewer {
public:

    Viewer();

    void launch();

    void setInteractionSource(InteractionSource *interactionSource);

    void setPCDSource(PCDSource *PCDSource);

private:
    igl::opengl::glfw::Viewer viewer;

    bool callBackPerDraw(igl::opengl::glfw::Viewer &viewer);

};


#endif //HOLOSCANNER_VIEWER_H
