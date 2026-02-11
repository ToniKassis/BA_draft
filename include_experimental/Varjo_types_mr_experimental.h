#ifndef VARJO_TYPES_MR_EXPERIMENTAL_H
#define VARJO_TYPES_MR_EXPERIMENTAL_H

#include "Varjo_types.h"
#include "Varjo_types_mr.h"

/*
 * *************** WARNING / DISCLAIMER ******************
 * Using these values is not guaranteed to work in the future. Please consult the MR team before using any of these.
 */

#if defined __cplusplus
extern "C" {
#endif

/*
 * Experimental API error codes
 */

/**
 * Shader error codes.
 */
static const varjo_Error varjo_Error_InvalidShaderType = 10100;
static const varjo_Error varjo_Error_InvalidShaderFormat = 10101;
static const varjo_Error varjo_Error_InvalidInputLayout = 10102;
static const varjo_Error varjo_Error_InvalidComputeBlockSize = 10103;
static const varjo_Error varjo_Error_InvalidSamplingMargin = 10104;
static const varjo_Error varjo_Error_InvalidConstantBufferSize = 10105;
static const varjo_Error varjo_Error_InvalidTextureDimensions = 10106;
static const varjo_Error varjo_Error_InvalidTextureFormat = 10107;
static const varjo_Error varjo_Error_InvalidTextureIndex = 10108;
static const varjo_Error varjo_Error_TextureNotAcquired = 10109;
static const varjo_Error varjo_Error_TexturesLocked = 10110;
static const varjo_Error varjo_Error_InvalidConstantBuffer = 10111;
static const varjo_Error varjo_Error_RenderAPINotInitialized = 10112;
static const varjo_Error varjo_Error_InvalidShaderFlags = 10113;
static const varjo_Error varjo_Error_InvalidShaderSize = 10114;
static const varjo_Error varjo_Error_InvalidShader = 10115;
static const varjo_Error varjo_Error_InvalidIndexCount = 10116;
static const varjo_Error varjo_Error_TextureLockFailed = 10117;

/**
 * Video Depth Test error codes
 */
static const varjo_Error varjo_Error_InvalidVideoDepthTestMode = 10201;
static const varjo_Error varjo_Error_InvalidVideoDepthTestRange = 10202;

/**
 * 3D reconstruction error codes.
 */
static const varjo_Error varjo_Error_ChunkContentsBufferAlreadyLocked = 10714;
static const varjo_Error varjo_Error_ChunkContentsBufferNotLocked = 10715;
static const varjo_Error varjo_Error_ChunkContentsBufferInvalidId = 10716;
static const varjo_Error varjo_Error_PointCloudSnapshotInvalidId = 10717;


/*
 * Experimental API lock types
 */
static const varjo_LockType varjo_LockType_VideoPostProcessShader = 3;  //!< Lock for video pass through post processing shader.
static const varjo_LockType varjo_LockType_VideoDepthTest = 4;          //!< Lock for video depth test modes and ranges.

/*
 * Documentation for varjo_MRGetCameraPropertyValues().
 * - Eye reprojection: Boolean. On/off. There is a physical offset between the users eyes and the video pass through cameras,
 *                     which can cause the perception of the real world to feel different than without the HMD.
 *                     When the eye reprojection is turned on, the software reprojects the video pass through
 *                     image from the physical camera position into the physical position of the users eyes.
 *                     While this will improve the perception it can also cause visual artifacts and therefore
 *                     can be unwanted behavior for some applications.
 *
 * Documentation for varjo_MRSetVRViewOffset():
 *
 * Note: This setting is ignored if eye reprojection is enabled (#varjo_CameraPropertyType_EyeReprojection). In this case
 * the rendering is always done from the users eye position (full VR position, corresponds to 'percentage' 0.0).
 */
static const varjo_CameraPropertyType varjo_CameraPropertyType_EyeReprojection = 6;  //!< Reproject the image to correspond the physical position of users eyes.


// =======================================================
// ============= Global Video Depth Test API =============
// =======================================================

/**
 * Video depth test mode constants.
 */
typedef int64_t varjo_VideoDepthTestMode;

/**
 * Video depth test range is not limited.
 */
static const varjo_VideoDepthTestMode varjo_VideoDepthTestMode_Full = 0;
/**
 * Video depth test is limited to given range if no application provides
 * depth test range through #varjo_ViewExtensionDepthTestRange.
 */
static const varjo_VideoDepthTestMode varjo_VideoDepthTestMode_LimitedRange = 1;
/**
 * Depth testing is forced and limited to given range. Depth test is done against
 * range farZ if application depth is not available. This mode automatically enables
 * video pass through rendering and video depth estimation to enable using it with
 * non-mixed reality applications.
 */
static const varjo_VideoDepthTestMode varjo_VideoDepthTestMode_ForcedRange = 2;

/**
 * Video depth test behavior constants.
 */
typedef int64_t varjo_VideoDepthTestBehavior;

/**
 * Prefer depth test range from application layer over video range. If any application layer
 * has set depth test range through #varjo_ViewExtensionDepthTestRange, then video depth test
 * range is ignored.
 */
static const varjo_VideoDepthTestBehavior varjo_VideoDepthTestBehavior_PreferLayerRange = 0;
/**
 * Prefer video depth test range over application layer ranges. Any application layer
 * specific depth test
 */
static const varjo_VideoDepthTestBehavior varjo_VideoDepthTestBehavior_PreferVideoRange = 1;
/**
 * Combine both video depth test range and application layer ranges. If both ranges are
 * provided, then stricter value will apply.
 */
static const varjo_VideoDepthTestBehavior varjo_VideoDepthTestBehavior_CombineRanges = 2;


// =========================================================
// ============= Video Post Process Shader API =============
// =========================================================

/**
 * Post process shader type constants.
 */
typedef int64_t varjo_ShaderType;
static const varjo_ShaderType varjo_ShaderType_VideoPostProcess = 1;

/**
 * Post process shader format constants.
 */
typedef int64_t varjo_ShaderFormat;
static const varjo_ShaderFormat varjo_ShaderFormat_None = 0;
static const varjo_ShaderFormat varjo_ShaderFormat_DxComputeBlob = 1;

/**
 * Post process shader input layout versioning for shader types.
 */
typedef int64_t varjo_ShaderInputLayout;

/**
 * This layout has these built-in inputs and outputs:
 * - Input RGBA texture that has the current video pass through image.
 * - Output RGBA texture that should be written from the shader.
 * - Constant buffer with the following data:
 *       - Texture dimensions.
 *
 *       - Source texture timestamp.
 *
 *       - Currently rendered view index: 0 Left context, 1 Right Context,
 *         2 Left focus, 3 Right focus.
 *
 *       - Destination rect. Area to render during a particular call to the shader.
 *         Source texture is guarateed to have have up-to-date data on this area.
 *
 *       - Projection and inverse projection matrices. Projection matrix used when
 *         the view was rendered.
 *
 *       - View and inverse view matrices. View matrix used when the view was rendered
 *         View matrix in this case is the inverse world pose of the HMD + eye offset.
 *
 *       - Source focus rect: The area of the focus view within the context view.
 *         x+y components denote the top left corner and w+z the bottom right corner.
 *         The coordinates are in absolute pixel values that are in the source context
 *         coordinate system. i.e. top left and bottom right are between (0,0)
 *         and (sourceContextSize.x, sourceContextSize.y).
 *         NOTE: When rendering a focus view, this is NOT the same as "sourceSize".
 *         Source size is the texture size, but due to distortions, projection and PPD
 *         differences, the effectively used source resolution for the focus view is
 *         less than the projected texture has.
 *
 *       - Source context size: Size of the source context texture. When rendering
 *         a context view this equals to "sourceSize".
 * - Linear clamping texture sampler
 * - Linear wrapping texture sampler
 *
 * HLSL register definitions:
 *
 * // Input buffer
 * Texture2D<float4> inputTex : register(t0);
 * // Output buffer
 * RWTexture2D<float4> outputTex : register(u0);
 * // Varjo generic constants
 * cbuffer ConstantBuffer : register(b0)
 * {
 *     int2 sourceSize;             // Source texture dimensions
 *     float sourceTime;            // Source texture timestamp
 *     int viewIndex;               // View to be rendered: 0=LC, 1=RC, 2=LF, 3=RF
 *     int4 destRect;               // Destination rectangle: x, y, w, h
 *     float4x4 projection;         // Projection matrix used for the source texture
 *     float4x4 inverseProjection;  // Inverse projection matrix
 *     float4x4 view;               // View matrix used for the source texture
 *     float4x4 inverseView;        // Inverse view matrix
 *     int4 sourceFocusRect;        // Area of the focus view within the context texture
 *     int2 sourceContextSize;      // Context texture size
 *     int2 padding;                // Unused
 * }
 *
 * // Varjo generic texture samplers
 * SamplerState SamplerLinearClamp : register(s0);
 * SamplerState SamplerLinearWrap : register(s1);
 */
static const varjo_ShaderInputLayout varjo_ShaderInputLayout_VideoPostProcess_V2 = 2;

/**
 * Post process shader input flags.
 */
typedef int64_t varjo_ShaderFlags_VideoPostProcess;
static const varjo_ShaderFlags_VideoPostProcess varjo_ShaderFlag_VideoPostProcess_None = 0;

struct varjo_TextureConfig {
    varjo_TextureFormat format;
    uint64_t width;
    uint64_t height;
};

/**
 * Post process shader info structure defining shader properties and inputs
 * for video post process shader.
 */
struct varjo_ShaderParams_VideoPostProcess {
    varjo_ShaderFlags_VideoPostProcess inputFlags;  //!< Shader input flags.
    int64_t computeBlockSize;                       //!< Compute shader block size. Valid values: 8 or 16.
    int64_t samplingMargin;                         //!< Amount of lines left as margin for sampling kernel. Valid values [0, 64].
    int64_t constantBufferSize;                     //!< Constant buffer size in bytes. Must be divisible by 16.
    struct varjo_TextureConfig textures[16];        //!< Input texture configurations.
};

/**
 * Wrapper for different types of shader parameters.
 */
union varjo_ShaderParams {
    struct varjo_ShaderParams_VideoPostProcess videoPostProcess;  //!< Parameters for varjo_ShaderType_VideoPostProcess.
    int64_t reserved[128];                                        //!< Reserved for future use.
};

/**
 * Shader configuration structure.
 */
struct varjo_ShaderConfig {
    varjo_ShaderFormat format;            //!< Shader format.
    varjo_ShaderInputLayout inputLayout;  //!< Shader input layout version.
    union varjo_ShaderParams params;      //!< Shader parameters.
};

/*
 * DXGI texture format type. This is DXGI_FORMAT type, but we don't want to include dxghi.h for this here.
 */
typedef uint32_t varjo_DXGITextureFormat;

/*
 * OpenGL texture format structure. Formats are GLenum type, but we don't want to include GL.h for this here.
 */
struct varjo_GLTextureFormat {
    uint32_t baseFormat;      //!< Base texture format (GLenum type)
    uint32_t internalFormat;  //!< Internal texture format (GLenum type)
};

// =====================================================================
// ================= 3D reconstruction point cloud API =================
// =====================================================================

/**
 * 3D reconstruction constants.
 */
typedef int32_t varjo_ChunkContentsBufferId;
static const varjo_ChunkContentsBufferId varjo_ChunkContentsBufferId_Invalid = -1;

/**
 * 3D reconstruction status properties.
 */
static const varjo_PropertyKey varjo_PropertyKey_ReconstructionAvailable = 0xFF01;  //!< boolean. Is 3D reconstruction capable hardware present.

/**
 * Point cloud snapshot id.
 */
typedef int64_t varjo_PointCloudSnapshotId;
static const int64_t varjo_PointCloudSnapshotId_Invalid = 0;

/**
 * Point cloud snapshot computation status.
 */
typedef int64_t varjo_PointCloudSnapshotStatus;
static const int64_t varjo_PointCloudSnapshotStatus_Invalid = 0;  //!< Snapshot does not exist or has been released.
static const int64_t varjo_PointCloudSnapshotStatus_Pending = 1;  //!< Snapshot capture has not yet finished.
static const int64_t varjo_PointCloudSnapshotStatus_Ready = 2;    //!< Snapshot is ready and can be accessed with GetPointCloudSnapshotContent.

/**
 * Environment 3D reconstruction configuration parameters.
 */
struct varjo_ReconstructionConfig {
    int32_t framerate;  //!< Target number of reconstruction updates per second.
    int32_t reserved[31];
};

/**
 * Data point which belongs to the 3D reconstruction. Contains the following fields in a packed format:
 *
 * Index: globally unique identifier for the point.
 * Confidence: Integer confidence value for the point. Points with confidence of 0 are to be considered removed.
 * Normal: Unit normal in world coordinates.
 * Color: RGB color of the point.
 * Position: position relative to the HMD tracking origin in meters.
 * Radius: Radius of the point in meters.
 */
struct varjo_PointCloudPoint {
    uint32_t indexConfidence;  //!< (index << 8) | (confidence & 0xFF);
    uint32_t normalXY;         //!< float16 / float16
    uint32_t normalZcolorR;    //!< float16 / float16
    uint32_t colorBG;          //!< float16 / float16
    uint32_t positionXY;       //!< float16 / float16
    uint32_t positionZradius;  //!< float16 / float16
};

/**
 * Point cloud snapshot content. Contains state of the point cloud at one moment in time.
 */
struct varjo_PointCloudSnapshotContent {
    struct varjo_PointCloudPoint* points;  //!< Array of points.
    int32_t pointCount;                    //!< Number of points in the array.
    int64_t timestamp;                     //!< Timestamp for the snapshot measured in reconstruction iterations.
};

/**
 * Point cloud delta content. Contains changes in point cloud which have occurred after previous snapshot or delta.
 */
struct varjo_PointCloudDeltaContent {
    struct varjo_PointCloudPoint* changedPoints;  //!< Array of points which have been updated.
    int32_t* removedPointIds;                     //!< Array of point ids which have been removed.
    int32_t changedPointCount;                    //!< Number of points in changedPoints array.
    int32_t removedPointCount;                    //!< Number of removed points in removedPointIds array.
    int32_t maxSurfelIndex;                       //!< Maximum value of index field in any of the points.
    int64_t timestamp;                            //!< Timestamp for delta measured in reconstruction iterations.
};

// =================================================================
// ================= 3D reconstruction meshing API =================
// =================================================================

/**
 * 3D reconstruction mesh constants.
 */
typedef int64_t varjo_MeshChunkContentsBufferId;
static const varjo_MeshChunkContentsBufferId varjo_MeshChunkContentsBufferId_Invalid = -1;


typedef int64_t varjo_VertexAttribute;
// static const varjo_VertexAttribute varjo_VertexAttribute_off = 0;
static const varjo_VertexAttribute varjo_VertexAttribute_float32 = 1;
// static const varjo_VertexAttribute varjo_VertexAttribute_float16 = 2;
// static const varjo_VertexAttribute varjo_VertexAttribute_rgbe = 3;

/**
 * Mesh vertex attribute formats.
 */
struct varjo_VertexFormat {
    varjo_VertexAttribute position;
    varjo_VertexAttribute color;
    varjo_VertexAttribute normal;
    int64_t reserved[13];
};

/**
 * Environment 3D reconstruction mesh configuration parameters.
 */
struct varjo_MeshReconstructionConfig {
    struct varjo_VertexFormat vertexFormat;  //!< Mesh vertex attribute formats.
    uint32_t chunksPerMeter;                 //!< Chunk resolution of x chunks per meter means each cubic meter is divided into x^3 chunks.
    uint32_t maxVerticesPerChunk;            //!< Maximum number of vertices a mesh chunk may contain.
    uint32_t maxTrianglesPerChunk;           //!< Maximum number of triangles a mesh chunk may contain.
    uint32_t maxChunks;                      //!< Maximum number of non-empty chunks that may exist simultaneously.
    uint32_t reserved[13];
};

/**
 * Environment 3D reconstruction mesh chunk description.
 *
 * The 3D reconstruction is subdivided into a grid of chunks.
 * The resolution of the grid may be retrieved with #varjo_MRGetMeshReconstructionConfig.
 * Chunk contents may be accessed using #varjo_MRLockMeshChunkContentsBuffer.
 */
struct varjo_MeshChunkDescription {
    struct varjo_Vector3Di position;    //!< Chunk position inside an integer grid. Real-world resolution defined by #varjo_MeshReconstructionConfig.
    varjo_Nanoseconds updateTimestamp;  //!< Timestamp the chunk last changed.
    uint32_t vertexCount;               //!< Number of vertices in the mesh chunk.
    uint32_t triangleCount;             //!< Number of triangles in the mesh chunk.
    uint32_t reserved[34];
};

/**
 * Supported attribute formats for mesh vertex colors.
 */
union varjo_MeshVertexPositionArray {
    struct varjo_Vector3Df* positions32f;
};

/**
 * Supported attribute formats for mesh vertex normals.
 */
union varjo_MeshVertexColorArray {
    struct varjo_Vector3Df* colors32f;
};

/**
 * Supported attribute formats for mesh vertex normals.
 */
union varjo_MeshVertexNormalArray {
    struct varjo_Vector3Df* normals32f;
};

/**
 * Contents of a 3D reconstruction chunks.
 */
struct varjo_MeshChunkContent {
    //! Same description which can be fetched with varjo_getChunkDescriptions. Duplicated here, since the reconstruction would run in an
    //! independent thread and the chunk could therefore change between fetching the description and contents.
    struct varjo_MeshChunkDescription description;
    uint32_t* triangleIndices;  //!< The i:th triangle is formed by vertices with indices triangleIndices[i*3], triangleIndices[i*3+1], triangleIndices[i*3+2].
    union varjo_MeshVertexPositionArray vertexPositions;  //!< Vertex positions in global coordinates. See #varjo_MeshReconstructionConfig for the format.
    union varjo_MeshVertexColorArray vertexColors;        //!< Vertex RGB color. See #varjo_MeshReconstructionConfig for the format.
    union varjo_MeshVertexNormalArray vertexNormals;      //!< Vertex normal vectors. See #varjo_MeshReconstructionConfig for the format.
    uint32_t reserved[18];
};

#if defined __cplusplus
}
#endif

#endif  // VARJO_TYPES_MR_EXPERIMENTAL_H
