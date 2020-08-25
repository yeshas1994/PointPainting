#pragma once

#include <Magnum/Mesh.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/Trade/MeshData3D.h>
#include <Magnum/MeshTools/Interleave.h>

namespace DosAutonomy
{
    using namespace Magnum;

    template <typename ShaderClass>
    bool createGLMeshFromMeshData(GL::Mesh& outMesh, const Trade::MeshData3D& meshData)
    {
        GL::Buffer vertices, indices;
        indices.setData(meshData.indices());
        if (meshData.hasNormals() && meshData.hasColors())
            vertices.setData(MeshTools::interleave(meshData.positions(0), meshData.normals(0), meshData.colors(0)));
        else if (meshData.hasNormals())
            vertices.setData(MeshTools::interleave(meshData.positions(0), meshData.normals(0)));
        else if (meshData.hasColors())
            vertices.setData(MeshTools::interleave(meshData.positions(0), meshData.colors(0)));
        else
            vertices.setData(meshData.positions(0));

        outMesh.setPrimitive(meshData.primitive())
               .setCount(indices.size())
               .setIndexBuffer(std::move(indices), 0, Magnum::MeshIndexType::UnsignedInt);
        if (meshData.hasNormals() && meshData.hasColors())
            outMesh.addVertexBuffer(std::move(vertices), 0, ShaderClass::Position{}, ShaderClass::Normal{}, ShaderClass::Color4{});
        else if (meshData.hasNormals())
            outMesh.addVertexBuffer(std::move(vertices), 0, ShaderClass::Position{}, ShaderClass::Normal{});
        else if (meshData.hasColors())
            outMesh.addVertexBuffer(std::move(vertices), 0, ShaderClass::Position{}, ShaderClass::Color4{});
        else
            outMesh.addVertexBuffer(std::move(vertices), 0, ShaderClass::Position{});

        return true;
    }

    template <typename ShaderClass>
    bool createGLMeshFromMeshDataNoNormals(GL::Mesh& outMesh, const Trade::MeshData3D& meshData)
    {
        GL::Buffer vertices, indices;
        indices.setData(meshData.indices());
        if (meshData.hasColors())
            vertices.setData(MeshTools::interleave(meshData.positions(0), meshData.colors(0)));
        else
            vertices.setData(meshData.positions(0));

        outMesh.setPrimitive(meshData.primitive())
            .setCount(indices.size())
            .setIndexBuffer(std::move(indices), 0, Magnum::MeshIndexType::UnsignedInt);
        if (meshData.hasColors())
            outMesh.addVertexBuffer(std::move(vertices), 0, ShaderClass::Position{}, ShaderClass::Color4{});
        else
            outMesh.addVertexBuffer(std::move(vertices), 0, ShaderClass::Position{});

        return true;
    }
}