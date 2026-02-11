// Copyright 2020 Varjo Technologies Oy. All rights reserved.
#ifndef VARJO_MR_EXPERIMENTAL_H
#define VARJO_MR_EXPERIMENTAL_H

#include "Varjo_export.h"
#include "Varjo_types.h"
#include "Varjo_types_mr.h"
#include "Varjo_types_mr_experimental.h"

#if defined __cplusplus
extern "C" {
#endif

// =======================================================
// ============= Global Video Depth Test API =============
// =======================================================

/**
 * Reset video pass-through layer depth test mode and range to defaults.
 *
 * Note: #varjo_LockType_VideoDepthTest lock must be held for this function to succeed. Use #varjo_Lock to obtain the lock.
 *
 * By default, #varjo_VideoDepthTestMode_LimitedRange mode and #varjo_VideoDepthTestBehavior_PreferLayerRange behavior are used.
 * The default video depth test range is 0..0.75m.
 *
 * Possible errors:
 *      #varjo_Error_OperationFailed, #varjo_Error_NotAvailable, #varjo_Error_NotLocked
 *
 * @param session Varjo session handle.
 */
VARJO_EXPERIMENTAL_API void varjo_MRResetVideoDepthTest(struct varjo_Session* session);

/**
 * Set video pass-through layer depth test mode.
 *
 * Note: #varjo_LockType_VideoDepthTest lock must be held for this function to succeed. Use #varjo_Lock to obtain the lock.
 *
 * See #varjo_VideoDepthTestMode for available modes.
 * See #varjo_VideoDepthTestBehavior for available range combination behaviors.
 *
 * If #varjo_VideoDepthTestMode_ForcedRange mode is used, video pass-through rendering
 * and video depth occlusion are also globally enabled to allow this to be used with
 * pure VR applications without any mixed reality support.
 *
 * Possible errors:
 *      #varjo_Error_OperationFailed, #varjo_Error_NotAvailable, #varjo_Error_NotLocked
 *
 * @param session Varjo session handle.
 * @param mode Video depth test operating mode.
 * @param behavior Video depth test range behavior with application layer depth test range.
 */
VARJO_EXPERIMENTAL_API void varjo_MRSetVideoDepthTestMode(struct varjo_Session* session, varjo_VideoDepthTestMode mode, varjo_VideoDepthTestBehavior behavior);

/**
 * Set video pass-through layer depth test range.
 *
 * Note: #varjo_LockType_VideoDepthTest lock must be held for this function to succeed. Use #varjo_Lock to obtain the lock.
 *
 * How ranges are used in different modes:
 * #varjo_VideoDepthTestMode_Full - Range limits are not used.
 * #varjo_VideoDepthTestMode_LimitedRange - Video pass-through depth test is limited to given range.
 * #varjo_VideoDepthTestMode_ForcedRange - As previous, but farZ is also used as fixed depth for layers not providing depth.
 *
 * Possible errors:
 *      #varjo_Error_OperationFailed, #varjo_Error_NotAvailable, #varjo_Error_NotLocked
 *
 * @param session Varjo session handle.
 * @param nearZ Video pass-through depth test range near limit.
 * @param farZ Video pass-through depth test range far limit.
 */
VARJO_EXPERIMENTAL_API void varjo_MRSetVideoDepthTestRange(struct varjo_Session* session, double nearZ, double farZ);


// =========================================================
// ============= Video Post Process Shader API =============
// =========================================================

/**
 * Turn shader on/off.
 *
 * Note: #varjo_LockType_VideoPostProcessShader lock must be held for this function to succeed. Use #varjo_Lock to obtain the lock.
 * Releasing the lock will disable the shader and reset post process shader state.
 *
 * Possible errors:
 *      #varjo_Error_OperationFailed, #varjo_Error_NotAvailable, #varjo_Error_NotLocked, #varjo_Error_InvalidShaderType
 *
 * @param session Varjo session handle.
 * @param shaderType Shader type to be toggled on/off.
 * @param enabled Is shader to be enabled or disabled.
 */
VARJO_EXPERIMENTAL_API void varjo_MRSetShader(struct varjo_Session* session, varjo_ShaderType shaderType, varjo_Bool enabled);

/**
 * Configure the shader and the input layout with D3D11 texture support.
 *
 * Note: #varjo_LockType_VideoPostProcessShader lock must be held for this function to succeed. Use #varjo_Lock to obtain the lock.
 * Releasing the lock will disable the shader and reset post process shader state.
 *
 * Possible errors:
 *      #varjo_Error_OperationFailed, #varjo_Error_NotAvailable, #varjo_Error_NotLocked, #varjo_Error_RenderAPINotInitialized,
 *      #varjo_Error_InvalidShaderType, #varjo_Error_InvalidShaderFormat, #varjo_Error_InvalidInputLayout,
 *      #varjo_Error_InvalidComputeBlockSize, #varjo_Error_InvalidSamplingMargin, #varjo_Error_InvalidConstantBufferSize,
 *      #varjo_Error_InvalidTextureDimensions, #varjo_Error_InvalidTextureFormat, #varjo_Error_InvalidShaderSize, #varjo_Error_InvalidShader
 *
 * @param session Varjo session handle.
 * @param device D3D11 device to be used.
 * @param shaderType Shader type.
 * @param config Shader config pointer. See #varjo_ShaderConfig for more details.
 * @param shaderData Pointer to shader data. Data format is specified in varjo_ShaderConfig::format.
 * @param shaderSize Shader data size in bytes.
 */
VARJO_EXPERIMENTAL_API void varjo_MRD3D11ConfigureShader(struct varjo_Session* session, struct ID3D11Device* device, varjo_ShaderType shaderType,
    const struct varjo_ShaderConfig* config, const char* shaderData, int32_t shaderSize);

/**
 * Configure the shader and the input layout with D3D12 texture support.
 *
 * Note: #varjo_LockType_VideoPostProcessShader lock must be held for this function to succeed. Use #varjo_Lock to obtain the lock.
 * Releasing the lock will disable the shader and reset post process shader state.
 *
 * Possible errors:
 *      #varjo_Error_OperationFailed, #varjo_Error_NotAvailable, #varjo_Error_NotLocked, #varjo_Error_RenderAPINotInitialized,
 *      #varjo_Error_InvalidShaderType, #varjo_Error_InvalidShaderFormat, #varjo_Error_InvalidInputLayout,
 *      #varjo_Error_InvalidComputeBlockSize, #varjo_Error_InvalidSamplingMargin, #varjo_Error_InvalidConstantBufferSize,
 *      #varjo_Error_InvalidTextureDimensions, #varjo_Error_InvalidTextureFormat, #varjo_Error_InvalidShaderSize, #varjo_Error_InvalidShader
 *
 * @param session Varjo session handle.
 * @param commandQueue D3D12 command queue to be used.
 * @param shaderType Shader type.
 * @param config Shader config pointer. See #varjo_ShaderConfig for more details.
 * @param shaderData Pointer to shader data. Data format is specified in varjo_ShaderConfig::format.
 * @param shaderSize Shader data size in bytes.
 */
VARJO_EXPERIMENTAL_API void varjo_MRD3D12ConfigureShader(struct varjo_Session* session, struct ID3D12CommandQueue* commandQueue, varjo_ShaderType shaderType,
    const struct varjo_ShaderConfig* config, const char* shaderData, int32_t shaderSize);

/**
 * Configure the shader and the input layout with OpenGL texture support.
 *
 * Note: #varjo_LockType_VideoPostProcessShader lock must be held for this function to succeed. Use #varjo_Lock to obtain the lock.
 * Releasing the lock will disable the shader and reset post process shader state.
 *
 * Possible errors:
 *      #varjo_Error_OperationFailed, #varjo_Error_NotAvailable, #varjo_Error_NotLocked, #varjo_Error_RenderAPINotInitialized,
 *      #varjo_Error_InvalidShaderType, #varjo_Error_InvalidShaderFormat, #varjo_Error_InvalidInputLayout,
 *      #varjo_Error_InvalidComputeBlockSize, #varjo_Error_InvalidSamplingMargin, #varjo_Error_InvalidConstantBufferSize,
 *      #varjo_Error_InvalidTextureDimensions, #varjo_Error_InvalidTextureFormat, #varjo_Error_InvalidShaderSize, #varjo_Error_InvalidShader
 *
 * @param session Varjo session handle.
 * @param shaderType Shader type.
 * @param config Shader config pointer. See #varjo_ShaderConfig for more details.
 * @param shaderData Pointer to shader data. Data format is specified in varjo_ShaderConfig::format.
 * @param shaderSize Shader data size in bytes.
 */
VARJO_EXPERIMENTAL_API void varjo_MRGLConfigureShader(
    struct varjo_Session* session, varjo_ShaderType shaderType, const struct varjo_ShaderConfig* config, const char* shaderData, int32_t shaderSize);

/**
 * Acquires write lock of a shader input texture.
 *
 * To be able to access the textures the shader must have been configured shader to use textures (varjo_MRxxxConfigureShader calls).
 *
 * Note: #varjo_LockType_VideoPostProcessShader lock must be held for this function to succeed. Use #varjo_Lock to obtain the lock.
 *
 * This must be called prior to texture update. The returned texture handle is only valid between
 * this call and #varjo_MRReleaseShaderTexture and should therefore be discarded after the release.
 *
 * Possible errors:
 *      #varjo_Error_OperationFailed, #varjo_Error_NotAvailable, #varjo_Error_NotLocked, #varjo_Error_InvalidShaderType,
 *      #varjo_Error_InvalidTextureIndex #varjo_Error_TextureLockFailed
 *
 * @param session Varjo session handle.
 * @param shaderType Shader type.
 * @param index Index of shader's texture input slot.
 * @return Locked texture handle. Use #varjo_ToD3D11Texture to cast into DX11 handle.
 */
VARJO_EXPERIMENTAL_API struct varjo_Texture varjo_MRAcquireShaderTexture(struct varjo_Session* session, varjo_ShaderType shaderType, int32_t index);

/**
 * Releases the acquired input texture (#varjo_MRAcquireShaderTexture).
 *
 * Note: #varjo_LockType_VideoPostProcessShader lock must be held for this function to succeed. Use #varjo_Lock to obtain the lock.
 *
 * After releasing the corresponding varjo_Texture should not be used anymore.
 *
 * Possible errors:
 *      #varjo_Error_OperationFailed, #varjo_Error_NotAvailable, #varjo_Error_NotLocked, #varjo_Error_InvalidShaderType,
 *      #varjo_Error_InvalidTextureIndex, #varjo_Error_TextureNotAcquired
 *
 * @param session Varjo session handle.
 * @param shaderType Shader type.
 * @param index Index of shader's texture input slot.
 */
VARJO_EXPERIMENTAL_API void varjo_MRReleaseShaderTexture(struct varjo_Session* session, varjo_ShaderType shaderType, int32_t index);

/**
 * Submits updates made to input textures and/or constant buffer.
 *
 * Note: #varjo_LockType_VideoPostProcessShader lock must be held for this function to succeed. Use #varjo_Lock to obtain the lock.
 *
 * This function applies clients changes to Varjo stack.
 * It does not need to be called on every frame necesssarily: only when the inputs need to be updated.
 *
 * Updates all provided inputs (textures and constant buffer) atomically.
 * Multiple calls to this function can also be used to update indices and/or constant buffer
 * separately and therefore non-atomically.
 *
 * If any of the textures being submitted is currently in locked state (acquired, but not released), this call fails.
 *
 * Possible errors:
 *      #varjo_Error_OperationFailed, #varjo_Error_NotAvailable, #varjo_Error_NotLocked, #varjo_Error_InvalidShaderType,
 *      #varjo_Error_TexturesLocked, #varjo_Error_InvalidTextureIndex, #varjo_Error_InvalidConstantBufferSize
 *
 * @param session Varjo session handle.
 * @param shaderType Shader type.
 * @param textureIndices List of texture input slot indices that have been updated. Can be 'nullptr' if no texture update is needed.
 * @param numTextureIndices Number of texture indices. Maximum number is the amount of shader texture slots.
 * @param constantBufferData Byte array of constant buffer data. Can be 'nullptr' if no constant buffer update is needed.
 * @param constantBufferSize Number of bytes in constant buffer byte array.
 */
VARJO_EXPERIMENTAL_API void varjo_MRSubmitShaderInputs(struct varjo_Session* session, varjo_ShaderType shaderType, const int32_t* textureIndices,
    int32_t numTextureIndices, const char* constantBufferData, int32_t constantBufferSize);

/**
 * Get amount of supported texture formats.
 *
 * Possible errors: #varjo_Error_InvalidShaderType
 *
 * @param session Varjo session handle.
 * @param renderAPI Render API type.
 * @param shaderType Shader type.
 * @return Number of supported texture formats.
 */
VARJO_EXPERIMENTAL_API int32_t varjo_MRGetSupportedShaderTextureFormatCount(
    struct varjo_Session* session, varjo_RenderAPI renderAPI, varjo_ShaderType shaderType);

/**
 * Get supported texture formats.
 *
 * Use #varjo_MRGetSupportedShaderTextureFormatCount to get the count of the formats.
 * Use #varjo_ToDXGIFormat and #varjo_ToGLFormat for getting native texture formats.
 *
 * Possible errors: #varjo_Error_InvalidShaderType, #varjo_Error_NullPointer
 *
 * @param session Varjo session handle.
 * @param renderAPI Render API type.
 * @param shaderType Shader type.
 * @param formats Output parameter for the list of supported formats.
 * @param maxSize Size of formats array.
 */
VARJO_EXPERIMENTAL_API void varjo_MRGetSupportedShaderTextureFormats(
    struct varjo_Session* session, varjo_RenderAPI renderAPI, varjo_ShaderType shaderType, varjo_TextureFormat* formats, int32_t maxSize);

/**
 * Converts varjo_TextureFormat to DXGI_FORMAT.
 *
 * Possible errors: #varjo_Error_InvalidTextureFormat
 *
 * @param session Varjo session handle.
 * @param format Varjo texture format to convert.
 * @return Corresponding DXGI_FORMAT or DXGI_FORMAT_UNKNOWN if the mapping does not exist.
 */
VARJO_EXPERIMENTAL_API varjo_DXGITextureFormat varjo_ToDXGIFormat(struct varjo_Session* session, varjo_TextureFormat format);

/**
 * Converts varjo_TextureFormat to GL format.
 *
 * Possible errors: #varjo_Error_InvalidTextureFormat
 *
 * @param session Varjo session handle.
 * @param format Varjo texture format to convert.
 * @return Corresponding GL texture formats or GL_NONE if the mapping does not exist.
 */
VARJO_EXPERIMENTAL_API struct varjo_GLTextureFormat varjo_ToGLFormat(struct varjo_Session* session, varjo_TextureFormat format);

// =====================================================================
// ================= 3D reconstruction point cloud API =================
// =====================================================================

/**
 * Start or stop 3D reconstruction.
 *
 * Possible errors: #varjo_Error_OperationFailed, #varjo_Error_NotAvailable, #varjo_Error_CapabilityNotAvailable
 *
 * @param session Varjo session handle.
 * @param enabled On/off toggle.
 */
VARJO_EXPERIMENTAL_API void varjo_MRSetReconstruction(struct varjo_Session* session, varjo_Bool enabled);

/**
 * Get currently applied 3D reconstruction configuration.
 *
 * Possible errors: #varjo_Error_OperationFailed, #varjo_Error_NotAvailable, #varjo_Error_CapabilityNotAvailable
 *
 * @param session Varjo session handle.
 */
VARJO_EXPERIMENTAL_API struct varjo_ReconstructionConfig varjo_MRGetReconstructionConfig(struct varjo_Session* session);

/**
 * Schedule a snapshot to be captured the next time point cloud is updated and start saving deltas.
 *
 * Possible errors: #varjo_Error_OperationFailed, #varjo_Error_NotAvailable, #varjo_Error_CapabilityNotAvailable
 *
 * @param session Varjo session handle.
 */
VARJO_EXPERIMENTAL_API varjo_PointCloudSnapshotId varjo_MRBeginPointCloudSnapshot(struct varjo_Session* session);

/**
 * Poll snapshot computation status.
 *
 * Possible errors: #varjo_Error_OperationFailed, #varjo_Error_NotAvailable, #varjo_Error_CapabilityNotAvailable
 *
 * @param session Varjo session handle.
 * @param id Id of the snapshot.
 */
VARJO_EXPERIMENTAL_API varjo_PointCloudSnapshotStatus varjo_MRGetPointCloudSnapshotStatus(struct varjo_Session* session, varjo_PointCloudSnapshotId id);

/**
 * Get pointer to snapshot content. Snapshot status must be varjo_PointCloudSnapshotStatus_Ready.
 *
 * Possible errors: #varjo_Error_OperationFailed, #varjo_Error_NotAvailable, #varjo_Error_CapabilityNotAvailable, #varjo_Error_NullPointer
 *
 * @param session Varjo session handle.
 * @param id Id of the snapshot.
 * @param content Output struct for snapshot content.
 */
VARJO_EXPERIMENTAL_API void varjo_MRGetPointCloudSnapshotContent(
    struct varjo_Session* session, varjo_PointCloudSnapshotId id, struct varjo_PointCloudSnapshotContent* content);

/**
 * Release snapshot. Deltas continue being saved until the snapshot is ended.
 *
 * Possible errors: #varjo_Error_OperationFailed, #varjo_Error_NotAvailable, #varjo_Error_CapabilityNotAvailable
 *
 * @param session Varjo session handle.
 * @param id Id of the snapshot.
 */
VARJO_EXPERIMENTAL_API void varjo_MRReleasePointCloudSnapshot(struct varjo_Session* session, varjo_PointCloudSnapshotId id);

/**
 * Ends the snapshot, deltas will no longer be saved. If snapshot or any deltas are unreleased, they are released.
 *
 * Possible errors: #varjo_Error_OperationFailed, #varjo_Error_NotAvailable, #varjo_Error_CapabilityNotAvailable
 *
 * @param session Varjo session handle.
 * @param id Id of the snapshot.
 */
VARJO_EXPERIMENTAL_API void varjo_MREndPointCloudSnapshot(struct varjo_Session* session, varjo_PointCloudSnapshotId id);

/**
 * Get the oldest delta that is built on top of the given snapshot and that has not yet been released.
 *
 * Possible errors: #varjo_Error_OperationFailed, #varjo_Error_NotAvailable, #varjo_Error_CapabilityNotAvailable, #varjo_Error_NullPointer
 *
 * @param session Varjo session handle.
 * @param id Id of the snapshot.
 * @param content Output struct for delta content.
 * @return False, if no new deltas are available yet.
 */
VARJO_EXPERIMENTAL_API varjo_Bool varjo_MRGetPointCloudDelta(
    struct varjo_Session* session, varjo_PointCloudSnapshotId id, struct varjo_PointCloudDeltaContent* content);

/**
 * Release the oldest delta that is built on top of the given snapshot.
 *
 * Possible errors: #varjo_Error_OperationFailed, #varjo_Error_NotAvailable, #varjo_Error_CapabilityNotAvailable
 *
 * @param session Varjo session handle.
 * @param id Id of the snapshot.
 */
VARJO_EXPERIMENTAL_API void varjo_MRPopPointCloudDelta(struct varjo_Session* session, varjo_PointCloudSnapshotId id);

// =================================================================
// ================= 3D reconstruction meshing API =================
// =================================================================

/**
 * Start or stop 3D reconstruction meshing.
 *
 * Possible errors: #varjo_Error_OperationFailed, #varjo_Error_NotAvailable, #varjo_Error_CapabilityNotAvailable
 *
 * @param session Varjo session handle.
 * @param enabled On/off toggle.
 */
VARJO_EXPERIMENTAL_API void varjo_MRSetMeshReconstruction(struct varjo_Session* session, varjo_Bool enabled);

/**
 * Get currently applied 3D reconstruction meshing configuration.
 *
 * Possible errors: #varjo_Error_OperationFailed, #varjo_Error_NotAvailable, #varjo_Error_CapabilityNotAvailable
 *
 * @param session Varjo session handle.
 */
VARJO_EXPERIMENTAL_API struct varjo_MeshReconstructionConfig varjo_MRGetMeshReconstructionConfig(struct varjo_Session* session);

/**
 * Get all reconstruction mesh chunks updated after a given timestamp.
 *
 * The 3D reconstruction is subdivided into a grid of chunks.
 * The resolution of the grid may be retrieved with #varjo_MRGetMeshReconstructionConfig.
 * Chunk contents may be accessed using #varjo_MRLockMeshChunkContentsBuffer.
 *
 * Possible errors:
 *     #varjo_Error_OperationFailed, #varjo_Error_NotAvailable, #varjo_Error_CapabilityNotAvailable
 *     #varjo_Error_InvalidSize
 *
 * @param session Varjo session handle.
 * @param updatedAfter Only chunks updated after this timestamp are returned. If 0, all non-empty chunks are returned.
 * @param buffer Output array for chunk descriptions.
 * @param bufferSize Size of the output buffer.
 * @param chunkCount Number of chunks written to the buffer.
 */
VARJO_EXPERIMENTAL_API void varjo_MRGetMeshChunkDescriptions(
    struct varjo_Session* session, varjo_Nanoseconds updatedAfter, struct varjo_MeshChunkDescription* buffer, int32_t bufferSize, int32_t* chunkCount);

/**
 * Lock mesh chunk contents buffer for reading.
 *
 * Possible errors:
 *     #varjo_Error_OperationFailed, #varjo_Error_NotAvailable, #varjo_Error_CapabilityNotAvailable
 *     #varjo_Error_ChunkContentsBufferAlreadyLocked, varjo_Error_InvalidSize
 *
 * @param session Varjo session handle.
 * @param positions Array of positions of the chunks to be locked.
 * @param count Size of the positions array.
 * @returns Chunk contents buffer id, may be used to access the contents using #varjo_MRGetMeshChunkContentsBufferData.
 */
VARJO_EXPERIMENTAL_API varjo_MeshChunkContentsBufferId varjo_MRLockMeshChunkContentsBuffer(
    struct varjo_Session* session, const struct varjo_Vector3Di* positions, int32_t count);

/**
 * Get mesh chunk contents from mesh chunk contents buffer.
 *
 * The buffer must be locked (#varjo_MRLockMeshChunkContentsBuffer) before calling this function.
 *
 * Possible errors:
 *     #varjo_Error_OperationFailed, #varjo_Error_NotAvailable, #varjo_Error_CapabilityNotAvailable
 *     #varjo_Error_ChunkContentsBufferNotLocked, #varjo_Error_ChunkContentsBufferInvalidId
 *
 * @param session Varjo session handle.
 * @param id Id of the already locked mesh chunk contents buffer.
 * @param chunkContents Output array for mesh chunk contents. Size must equal the count passed to #varjo_MRLockMeshChunkContentsBuffer.
 */
VARJO_EXPERIMENTAL_API void varjo_MRGetMeshChunkContentsBufferData(
    struct varjo_Session* session, varjo_MeshChunkContentsBufferId id, struct varjo_MeshChunkContent* chunkContents);

/**
 * Unlock already locked chunk contents buffer.
 *
 * Possible errors:
 *     #varjo_Error_OperationFailed, #varjo_Error_NotAvailable, #varjo_Error_CapabilityNotAvailable
 *     #varjo_Error_ChunkContentsBufferNotLocked, #varjo_Error_ChunkContentsBufferInvalidId
 *
 * @param session Varjo session handle.
 * @param id Id of the already locked mesh chunk contents buffer.
 */
VARJO_EXPERIMENTAL_API void varjo_MRUnlockMeshChunkContentsBuffer(struct varjo_Session* session, varjo_MeshChunkContentsBufferId id);

#if defined __cplusplus
}
#endif
#endif  // VARJO_MR_EXPERIMENTAL_H
