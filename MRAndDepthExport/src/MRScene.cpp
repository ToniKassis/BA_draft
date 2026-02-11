// Copyright 2019-2021 Varjo Technologies Oy. All rights reserved.

#include "MRScene.hpp"

#include "ExampleShaders.hpp"

// VarjoExamples namespace contains simple example wrappers for using Varjo API features.
// These are only meant to be used in SDK example applications. In your own application,
// use your own production quality integration layer.
using namespace VarjoExamples;

namespace
{
    // 1 unit in cubemap corresponds to this many cd/m^2
    constexpr double c_nitsPerUnit = 100.0;

    // Scene luminance constant to simulate proper lighting.
    constexpr double c_sceneLuminance = 196.0 / (3.0 * c_nitsPerUnit);

    // Vertex data for plane (position + uv)
    constexpr float d = 1.0f;
    constexpr float r = d * 0.5f;

    // unit cube centered at origin, size 1 (we’ll scale to length/radius in render)
    struct PNT {
        float px, py, pz;
        float nx, ny, nz;
    };

    std::unique_ptr<VarjoExamples::Renderer::Mesh>
        CreateUnitCubeMesh(VarjoExamples::Renderer& r)
    {
        // 8 positions, but for proper lighting/normals we use 24 verts (4 per face)
        static const PNT v[] = {
            // +Y face
            {-0.5f, 0.5f,-0.5f, 0,1,0}, { 0.5f, 0.5f,-0.5f, 0,1,0},
            { 0.5f, 0.5f, 0.5f, 0,1,0}, {-0.5f, 0.5f, 0.5f, 0,1,0},
            // -Y face
            {-0.5f,-0.5f, 0.5f, 0,-1,0}, { 0.5f,-0.5f, 0.5f, 0,-1,0},
            { 0.5f,-0.5f,-0.5f, 0,-1,0}, {-0.5f,-0.5f,-0.5f, 0,-1,0},
            // +X face
            { 0.5f,-0.5f,-0.5f, 1,0,0}, { 0.5f, 0.5f,-0.5f, 1,0,0},
            { 0.5f, 0.5f, 0.5f, 1,0,0}, { 0.5f,-0.5f, 0.5f, 1,0,0},
            // -X face
            {-0.5f,-0.5f, 0.5f,-1,0,0}, {-0.5f, 0.5f, 0.5f,-1,0,0},
            {-0.5f, 0.5f,-0.5f,-1,0,0}, {-0.5f,-0.5f,-0.5f,-1,0,0},
            // +Z face
            {-0.5f,-0.5f, 0.5f, 0,0,1}, { 0.5f,-0.5f, 0.5f, 0,0,1},
            { 0.5f, 0.5f, 0.5f, 0,0,1}, {-0.5f, 0.5f, 0.5f, 0,0,1},
            // -Z face
            { 0.5f,-0.5f,-0.5f, 0,0,-1}, {-0.5f,-0.5f,-0.5f, 0,0,-1},
            {-0.5f, 0.5f,-0.5f, 0,0,-1}, { 0.5f, 0.5f,-0.5f, 0,0,-1},
        };

        static const unsigned idx[] = {
            0,1,2, 0,2,3,    4,5,6, 4,6,7,
            8,9,10, 8,10,11, 12,13,14, 12,14,15,
            16,17,18, 16,18,19, 20,21,22, 20,22,23
        };

        std::vector<float> verts; verts.reserve(std::size(v) * 6);
        for (const auto& p : v) {
            verts.push_back(p.px); verts.push_back(p.py); verts.push_back(p.pz);
            verts.push_back(p.nx); verts.push_back(p.ny); verts.push_back(p.nz);
        }
        std::vector<unsigned> indices(idx, idx + std::size(idx));

        return r.createMesh(verts, sizeof(float) * 6, indices, VarjoExamples::Renderer::PrimitiveTopology::TriangleList);
    }

    const std::vector<float> c_planeVertexData = {
        -2 * r, -2 * r, 0, 0, 1,
        -2 * r,  2 * r, 0, 0, 0,
         2 * r, -2 * r, 0, 1, 1,
         2 * r,  2 * r, 0, 1, 0,
    };

    // Index data for plane
    const std::vector<unsigned int> c_planeIndexData = {
        0, 2, 1,
        1, 2, 3,
    };

}  // namespace

MRScene::MRScene(Renderer& renderer)
    : m_renderer(renderer)
    // NOTE: all cube-related meshes/shaders are intentionally omitted
    , m_texturedPlaneMesh(renderer.createMesh(
        c_planeVertexData, sizeof(float) * 5, c_planeIndexData, Renderer::PrimitiveTopology::TriangleList))
    , m_texturedPlaneShader(renderer.getShaders().createShader(ExampleShaders::ShaderType::TexturedPlane))
{
    // simple solid shader for untextured geometry
    m_boneShader = renderer.getShaders().createShader(ExampleShaders::ShaderType::SolidCube);

    // unit cube we’ll stretch/rotate per segment
    m_boneMesh = CreateUnitCubeMesh(renderer);

}

void MRScene::setSkeletonSegments(std::vector<Segment>&& segs) {
    m_segments = std::move(segs);
}

void MRScene::updateHdrCubemap(uint32_t resolution, varjo_TextureFormat format, size_t rowPitch, const uint8_t* data)
{
    // You can keep this as-is (no cubes will be rendered even if the cubemap exists).
    if (data) {
        // Create cubemap if not created or the resolution has changed.
        if (!m_hdrCubemapTexture || m_hdrCubemapTexture->getSize().x != resolution) {
            m_hdrCubemapTexture = m_renderer.createHdrCubemap(resolution, format);
        }

        if (m_hdrCubemapTexture) {
            m_renderer.updateTexture(m_hdrCubemapTexture.get(), data, rowPitch);
        }
    }
    else {
        m_hdrCubemapTexture.reset();
    }
}

void MRScene::updateColorFrame(int ch, const glm::ivec2& resolution, varjo_TextureFormat format, size_t rowPitch, const uint8_t* data)
{
    assert(ch >= 0 && ch < static_cast<int>(m_colorFrameTextures.size()));

    auto& colorFrameTexture = m_colorFrameTextures[ch];

    if (data) {
        // Create texture if not created or the resolution has changed.
        if (!colorFrameTexture || colorFrameTexture->getSize() != resolution) {
            colorFrameTexture = m_renderer.createTexture2D(resolution, format);
        }

        if (colorFrameTexture) {
            m_renderer.updateTexture(colorFrameTexture.get(), data, rowPitch);
        }
    }
    else {
        colorFrameTexture.reset();
    }
}

void MRScene::onUpdate(double frameTime, double /*deltaTime*/, int64_t /*frameCounter*/, const VarjoExamples::Scene::UpdateParams& updateParams)
{
    // Cast scene update params
    const auto& params = reinterpret_cast<const UpdateParams&>(updateParams);

    // Reset exposure gain when the brightness simulation is toggled. This is to ensure that the filter below won't
    // cause any flicker when switching back and forth.
    

    // Calculate exposure gain based on camera parameters, if brightness simulation is enabled.
    
    m_wbNormalization = params.cameraParams.wbNormalizationData;
    //m_prevSimulateBrightness = params.cameraParams.simulateBrightness;

    // Scale lighting with scene luminance.
    m_lighting = params.lighting;

    /*
    // Position textured planes (keep or modify as you like)
    for (size_t ch = 0; ch < m_texturedPlanes.size(); ch++) {
        m_texturedPlanes[ch].pose.position = { -0.75f + static_cast<float>(ch) * 1.5f, 1.5f, -1.8f };
        m_texturedPlanes[ch].pose.scale = { 0.5f, 0.5f, 0.5f };
        m_texturedPlanes[ch].color = { 1.0f, 1.0f, 1.0f, 1.0f };
        m_texturedPlanes[ch].vtxColorFactor = 0.0f;
    }
    */
}

void MRScene::onRender(
    Renderer& renderer,
    Renderer::ColorDepthRenderTarget& target,
    int viewIndex,
    const glm::mat4x4& viewMat,
    const glm::mat4x4& projMat,
    void* userData) const
{
    (void)target; (void)viewIndex; (void)userData;

  /*
    // Render only the textured planes
    for (size_t ch = 0; ch < m_texturedPlanes.size(); ch++) {
        auto& colorFrameTexture = m_colorFrameTextures[ch];
        if (colorFrameTexture) {
            renderer.bindShader(*m_texturedPlaneShader);
            renderer.bindTextures({ colorFrameTexture.get() });

            ExampleShaders::TexturedPlaneConstants constants{};

#if 1
            // Calculate model transformation
            glm::mat4x4 modelMat(1.0f);
            modelMat = glm::translate(modelMat, m_texturedPlanes[ch].pose.position);
            modelMat *= glm::toMat4(m_texturedPlanes[ch].pose.rotation);
            modelMat = glm::scale(modelMat, m_texturedPlanes[ch].pose.scale);

            // Render plane objects in the world
            constants.vs.transform = ExampleShaders::TransformData(modelMat, viewMat, projMat);
#else
            // For full view projection
            constants.vs.transform = ExampleShaders::TransformData(glm::mat4x4(1.0f), glm::mat4x4(1.0f), glm::mat4x4(1.0f));
#endif

            constants.ps.colorCorrection = glm::vec4(1.0, 1.0, 1.0, 1.0);

            renderer.renderMesh(*m_texturedPlaneMesh, constants.vs, constants.ps);
        }
    }
    */
    if (m_boneMesh && m_boneShader && !m_segments.empty()) {
        renderer.bindShader(*m_boneShader);

        for (const auto& s : m_segments) {
            const glm::vec3 a = s.a;
            const glm::vec3 b = s.b;
            glm::vec3 d = b - a;
            const float len = glm::length(d);
            if (len <= 1e-6f) continue;
            d /= len; // direction (unit)

            // build an orthonormal basis with d as local +Y axis
            glm::vec3 up = { 0,1,0 };
            if (std::abs(glm::dot(up, d)) > 0.99f) up = { 1,0,0 };
            glm::vec3 x = glm::normalize(glm::cross(up, d));  // right
            glm::vec3 z = glm::normalize(glm::cross(d, x));   // forward

            // rotation matrix columns (x, y=d, z)
            glm::mat4 R(1.0f);
            R[0] = glm::vec4(x, 0.0f);
            R[1] = glm::vec4(d, 0.0f); // align cube's local +Y with the segment
            R[2] = glm::vec4(z, 0.0f);

            // scale: thickness on X/Z, length on Y
            const float th = std::max(0.001f, s.radius * 2.0f); // cube is size 1 → use diameter
            glm::mat4 S = glm::scale(glm::mat4(1.0f), glm::vec3(th, len, th));

            // translate to midpoint
            glm::vec3 worldOffset(0.15f, -0.10f, 0.0f);
            glm::vec3 mid = (a + b) * 0.5f + worldOffset;
            glm::mat4 T = glm::translate(glm::mat4(1.0f), mid);

            glm::mat4 model = T * R * S;

            // constants: Solid shader matches TexturedPlane style in examples
            ExampleShaders::SolidCubeConstants c{};
            c.vs.transform = ExampleShaders::TransformData(model, viewMat, projMat);
            c.vs.objectColor = glm::vec4(0.1f, 0.9f, 0.2f, s.alpha); // green-ish, alpha from confidence

            renderer.renderMesh(*m_boneMesh, c.vs, c.ps);
        }
    }

}
