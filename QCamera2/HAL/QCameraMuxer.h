/* Copyright (c) 2015, The Linux Foundataion. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are
* met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above
*       copyright notice, this list of conditions and the following
*       disclaimer in the documentation and/or other materials provided
*       with the distribution.
*     * Neither the name of The Linux Foundation nor the names of its
*       contributors may be used to endorse or promote products derived
*       from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
* ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
* BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
* OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
* IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef __QCAMERAMUXER_H__
#define __QCAMERAMUXER_H__

#include <hardware/camera.h>
#include <system/camera.h>
#include "QCamera2HWI.h"

namespace qcamera {

#define MAX_NUM_CAMERA_PER_BUNDLE 4

/* Struct@ qcamera_physical_descriptor_t
 *
 *  Description@ This structure specifies various attributes
 *      physical cameras enumerated on the device
 */
typedef struct {
    // Userspace Physical Camera ID
    uint32_t id;
    // Server Camera ID
    uint32_t camera_server_id;
    // Device version
    uint32_t device_version;
    // Specifies type of camera
    cam_sync_type_t type;
    // Specifies mode of Camera
    cam_sync_mode_t mode;
    // Camera Info
    camera_info cam_info;
    // Reference to HWI
    QCamera2HardwareInterface *hwi;
    // Reference to camera device structure
    camera_device_t* dev;
} qcamera_physical_descriptor_t;

/* Struct@ qcamera_logical_descriptor_t
 *
 *  Description@ This structure stores information about logical cameras
 *      and corresponding data of the physical camera that are part of
 *      this logical camera
 */
typedef struct {
    // Camera Device to be shared to Frameworks
    camera_device_t dev;
    // Device version
    uint32_t device_version;
    // Logical Camera ID
    uint32_t id;
    // Logical Camera Facing
    int32_t facing;
    // Number of Physical camera present in this logical camera
    uint32_t numCameras;
    // To signify if the LINK/UNLINK established between physical cameras
    bool bSyncOn;
    // Signifies Physical Camera ID of each camera
    uint32_t pId[MAX_NUM_CAMERA_PER_BUNDLE];
    // Signifies server camera ID of each camera
    uint32_t sId[MAX_NUM_CAMERA_PER_BUNDLE];
    // Signifies type of each camera
    cam_sync_type_t type[MAX_NUM_CAMERA_PER_BUNDLE];
    // Signifies mode of each camera
    cam_sync_mode_t mode[MAX_NUM_CAMERA_PER_BUNDLE];
} qcamera_logical_descriptor_t;

/* Class@ QCameraMuxer
 *
 * Description@ Muxer interface
 *    a) Manages the grouping of the physical cameras into a logical camera
 *    b) Muxes the operational calls from Frameworks to HWI
 *    c) Composes MPO from JPEG
 */
class QCameraMuxer {

public:
    /* Public Methods   */
    QCameraMuxer(uint32_t num_of_cameras);
    virtual ~QCameraMuxer();
    static void getCameraMuxer(QCameraMuxer** pCamMuxer,
            uint32_t num_of_cameras);
    static int get_number_of_cameras();
    static int get_camera_info(int camera_id, struct camera_info *info);
    static int set_callbacks(const camera_module_callbacks_t *callbacks);
    static int open_legacy(const struct hw_module_t* module,
            const char* id, uint32_t halVersion, struct hw_device_t** device);

    static int camera_device_open(const struct hw_module_t* module,
            const char* id,
            struct hw_device_t** device);
    static int close_camera_device( hw_device_t *);

    /* Operation methods directly accessed by Camera Service */
    static camera_device_ops_t mCameraMuxerOps;

    /* Start of operational methods */
    static int set_preview_window(struct camera_device *,
            struct preview_stream_ops *window);
    static void set_callBacks(struct camera_device *,
            camera_notify_callback notify_cb,
            camera_data_callback data_cb,
            camera_data_timestamp_callback data_cb_timestamp,
            camera_request_memory get_memory,
            void *user);
    static void enable_msg_type(struct camera_device *, int32_t msg_type);
    static void disable_msg_type(struct camera_device *, int32_t msg_type);
    static int msg_type_enabled(struct camera_device *, int32_t msg_type);
    static int start_preview(struct camera_device *);
    static void stop_preview(struct camera_device *);
    static int preview_enabled(struct camera_device *);
    static int store_meta_data_in_buffers(struct camera_device *,
            int enable);
    static int start_recording(struct camera_device *);
    static void stop_recording(struct camera_device *);
    static int recording_enabled(struct camera_device *);
    static void release_recording_frame(struct camera_device *,
              const void *opaque);
    static int auto_focus(struct camera_device *);
    static int cancel_auto_focus(struct camera_device *);
    static int take_picture(struct camera_device *);
    static int cancel_picture(struct camera_device *);
    static int set_parameters(struct camera_device *, const char *parms);
    static char* get_parameters(struct camera_device *);
    static void put_parameters(struct camera_device *, char *);
    static int send_command(struct camera_device *,
          int32_t cmd, int32_t arg1, int32_t arg2);
    static void release(struct camera_device *);
    static int dump(struct camera_device *, int fd);
    /* End of operational methods */
    typedef void (*jpeg_data_callback)(int32_t msg_type,
            const camera_memory_t *data, unsigned int index,
            camera_frame_metadata_t *metadata, void *user);
    // Temporary implementation to handle both JPEG callbacks
    // This will be revisited
    static void jpeg1_data_callback(int32_t msg_type,
            const camera_memory_t *data, unsigned int index,
            camera_frame_metadata_t *metadata, void *user);
    static void jpeg2_data_callback(int32_t msg_type,
            const camera_memory_t *data, unsigned int index,
            camera_frame_metadata_t *metadata, void *user);

public:
    /* Public Members  Variables   */
    mm_jpeg_ops_t mJpegOps;
    uint32_t mJpegClientHandle;
    // Stores Camera Data Callback function
    camera_data_callback mDataCb;

private:
    /* Private Member Variables  */
    qcamera_physical_descriptor_t *mPhyCamera;
    qcamera_logical_descriptor_t *mLogicalCamera;
    const camera_module_callbacks_t *mCallbacks;
    bool bDualCameraEnabled;
    bool bAuxCameraExposed;
    uint8_t m_nPhyCameras;
    uint8_t m_nLogicalCameras;

    /* Private Member Methods */
    int setupLogicalCameras();
    int cameraDeviceOpen(int camera_id, struct hw_device_t **hw_device);
    int getNumberOfCameras();
    int getCameraInfo(int camera_id, struct camera_info *info,
            cam_sync_type_t *p_cam_type);
    int setCallbacks(const camera_module_callbacks_t *callbacks);
    int setDataCallback(camera_data_callback data_cb);
    qcamera_logical_descriptor_t* getLogicalCamera(
            struct camera_device * device);
    qcamera_physical_descriptor_t* getPhysicalCamera(
            qcamera_logical_descriptor_t* log_cam, uint32_t index);
    void setJpegHandle(uint32_t handle) { mJpegClientHandle = handle;};

};// End namespace qcamera

}
#endif /* __QCAMERAMUXER_H__ */

