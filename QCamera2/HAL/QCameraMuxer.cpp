/* Copyright (c) 2015, The Linux Foundation. All rights reserved.
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
 *
 */

#define ALOG_NIDEBUG 0
#define LOG_TAG "QCameraMuxer"
#include <utils/Log.h>
#include <utils/threads.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <binder/IMemory.h>
#include <binder/MemoryBase.h>
#include <binder/MemoryHeapBase.h>
#include <utils/RefBase.h>

#include "QCameraMuxer.h"
#include "QCamera2HWI.h"


/* Muxer implementation */

using namespace android;
namespace qcamera {

QCameraMuxer *gMuxer = NULL;

//Error Check Macros
#define CHECK_MUXER() \
    if (!gMuxer) { \
        ALOGE("%s[%d]: Error getting muxer ", __func__, __LINE__); \
        return; \
    } \

#define CHECK_MUXER_ERROR() \
    if (!gMuxer) { \
        ALOGE("%s[%d]: Error getting muxer ", __func__, __LINE__); \
        return -ENODEV; \
    } \

#define CHECK_CAMERA(pCam) \
    if (!pCam) { \
        ALOGE("%s[%d]: Error getting physical camera", __func__, __LINE__); \
        return; \
    } \

#define CHECK_CAMERA_ERROR(pCam) \
    if (!pCam) { \
        ALOGE("%s[%d]: Error getting physical camera", __func__, __LINE__); \
        return -ENODEV; \
    } \

#define CHECK_HWI(hwi) \
    if (!hwi) { \
        ALOGE("%s[%d]: Error !! HWI not found!!", __func__, __LINE__); \
        return; \
    } \

#define CHECK_HWI_ERROR(hwi) \
    if (!hwi) { \
        ALOGE("%s[%d]: Error !! HWI not found!!", __func__, __LINE__); \
        return -ENODEV; \
    } \


/*===========================================================================
 * FUNCTION         : getCameraMuxer
 *
 * DESCRIPTION     : Creates Camera Muxer if not created
 *
 * PARAMETERS:
 *   @pMuxer               : Pointer to retrieve Camera Muxer
 *   @num_of_cameras  : Number of Physical Cameras on device
 *
 * RETURN             :  NONE
 *==========================================================================*/
void QCameraMuxer::getCameraMuxer(
        QCameraMuxer** pMuxer, uint32_t num_of_cameras)
{
    *pMuxer = NULL;
    if (!gMuxer) {
        gMuxer = new QCameraMuxer(num_of_cameras);
    }
    CHECK_MUXER();
    *pMuxer = gMuxer;
    CDBG_HIGH("%s[%d]: gMuxer: %p ", __func__, __LINE__, gMuxer);
    return;
}

/*===========================================================================
 * FUNCTION         : QCameraMuxer
 *
 * DESCRIPTION     : QCameraMuxer Constructor
 *
 * PARAMETERS:
 *   @num_of_cameras  : Number of Physical Cameras on device
 *
 *==========================================================================*/
QCameraMuxer::QCameraMuxer(uint32_t num_of_cameras)
{
    mPhyCamera          = NULL;
    mLogicalCamera      = NULL;
    mCallbacks          = NULL;
    m_nPhyCameras       = num_of_cameras;
    mJpegClientHandle   = 0;
    setupLogicalCameras();
}

/*===========================================================================
 * FUNCTION         : ~QCameraMuxer
 *
 * DESCRIPTION     : QCameraMuxer Desctructor
 *
 *==========================================================================*/
QCameraMuxer::~QCameraMuxer() {
    if (mLogicalCamera) {
        delete [] mLogicalCamera;
        mLogicalCamera = NULL;
    }
    if (mPhyCamera) {
        delete [] mPhyCamera;
        mPhyCamera = NULL;
    }
}

/*===========================================================================
 * FUNCTION         : get_number_of_cameras
 *
 * DESCRIPTION     : Provide number of Logical Cameras
 *
 * RETURN             :  Number of logical Cameras
 *==========================================================================*/
int QCameraMuxer::get_number_of_cameras()
{
    return gMuxer->getNumberOfCameras();
}

/*===========================================================================
 * FUNCTION         : get_camera_info
 *
 * DESCRIPTION     : get logical camera info
 *
 * PARAMETERS:
 *   @camera_id     : Logical Camera ID
 *   @info              : Logical Main Camera Info
 *
 * RETURN     :
 *              NO_ERROR  : success
 *              ENODEV : Camera not found
 *              other: non-zero failure code
 *==========================================================================*/
int QCameraMuxer::get_camera_info(int camera_id, struct camera_info *info)
{
    int rc = NO_ERROR;
    CDBG_HIGH("%s: E", __func__);
    cam_sync_type_t type;
    if ((camera_id < 0) || (camera_id >= gMuxer->getNumberOfCameras())) {
        ALOGE("%s : Camera id %d not found!", __func__, camera_id);
        return -ENODEV;
    }
    if(info) {
        rc = gMuxer->getCameraInfo(camera_id, info, &type);
    }
    CDBG_HIGH("%s: X, rc: %d", __func__, rc);
    return rc;
}


/*===========================================================================
 * FUNCTION         : set_callbacks
 *
 * DESCRIPTION     : Not Implemented
 *
 * PARAMETERS:
 *   @callbacks      : Camera Module Callbacks
 *
 * RETURN     :
 *              NO_ERROR  : success
 *              other: non-zero failure code
 *==========================================================================*/
int QCameraMuxer::set_callbacks(const camera_module_callbacks_t *callbacks)
{
    // Not implemented
    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : camera_device_open
 *
 * DESCRIPTION: static function to open a camera device by its ID
 *
 * PARAMETERS :
 *   @modue: hw module
 *   @id : camera ID
 *   @hw_device : ptr to struct storing camera hardware device info
 *
 * RETURN     :
 *              NO_ERROR  : success
 *              BAD_VALUE : Invalid Camera ID
 *              other: non-zero failure code
 *==========================================================================*/
int QCameraMuxer::camera_device_open(
        const struct hw_module_t *module, const char *id,
        struct hw_device_t **hw_device)
{
    int rc = NO_ERROR;
    CDBG_HIGH("%s[%d]: id= %d", __func__, __LINE__, atoi(id));
    if (!id) {
        ALOGE("%s: Invalid camera id", __func__);
        return BAD_VALUE;
    }

    rc =  gMuxer->cameraDeviceOpen(atoi(id), hw_device);
    CDBG_HIGH("%s[%d]: id= %d, rc: %d", __func__, __LINE__, atoi(id), rc);
    return rc;
}

/*===========================================================================
 * FUNCTION   : open_legacy
 *
 * DESCRIPTION: static function to open a camera device by its ID
 *
 * PARAMETERS :
 *   @modue: hw module
 *   @id : camera ID
 *   @halVersion: hal version
 *   @hw_device : ptr to struct storing camera hardware device info
 *
 * RETURN     :
 *              NO_ERROR  : success
 *              BAD_VALUE : Invalid Camera ID
 *              other: non-zero failure code
 *==========================================================================*/
int QCameraMuxer::open_legacy(const struct hw_module_t* module,
        const char* id, uint32_t halVersion, struct hw_device_t** hw_device)
{
    int rc = NO_ERROR;
    CDBG_HIGH("%s[%d]: id= %d", __func__, __LINE__, atoi(id));
    if (!id) {
        ALOGE("%s: Invalid camera id", __func__);
        return BAD_VALUE;
    }

    rc =  gMuxer->cameraDeviceOpen(atoi(id), hw_device);
    CDBG_HIGH("%s[%d]: id= %d, rc: %d", __func__, __LINE__, atoi(id), rc);
    return rc;
}

/*===========================================================================
 * FUNCTION   : set_preview_window
 *
 * DESCRIPTION: Set Preview window for main camera
 *
 * PARAMETERS :
 *   @device : camera hardware device info
 *   @window: Preview window ops
 *
 * RETURN     :
 *              NO_ERROR  : success
 *              other: non-zero failure code
 *==========================================================================*/
int QCameraMuxer::set_preview_window(struct camera_device * device,
        struct preview_stream_ops *window)
{
    int rc = NO_ERROR;
    CHECK_MUXER_ERROR();
    qcamera_physical_descriptor_t *pCam = NULL;
    qcamera_logical_descriptor_t *cam = gMuxer->getLogicalCamera(device);
    CHECK_CAMERA_ERROR(cam);

    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA_ERROR(pCam);

        // Set preview window only for primary camera
        if (pCam->mode == CAM_MODE_PRIMARY) {
            QCamera2HardwareInterface *hwi = pCam->hwi;
            CHECK_HWI_ERROR(hwi);
            rc = hwi->set_preview_window(pCam->dev, window);
            if (rc != NO_ERROR) {
                ALOGE("%s: Error!! setting preview window", __func__);
                return rc;
            }
            break;
        }
    }
    return rc;
}

/*===========================================================================
 * FUNCTION   : set_callBacks
 *
 * DESCRIPTION: Set Framework callbacks to notify various frame data asynchronously
 *
 * PARAMETERS :
 *   @device : camera hardware device info
 *   @notify_cb: Notification callback
 *   @data_cb: data callback
 *   @data_cb_timestamp: data timestamp callback
 *   @get_memory: callback to obtain memory
 *   @user : userdata
 *
 * RETURN : None
 *==========================================================================*/
void QCameraMuxer::set_callBacks(struct camera_device * device,
        camera_notify_callback notify_cb,
        camera_data_callback data_cb,
        camera_data_timestamp_callback data_cb_timestamp,
        camera_request_memory get_memory,
        void *user)
{
    CDBG_HIGH("%s: E", __func__);
    CHECK_MUXER();
    qcamera_physical_descriptor_t *pCam = NULL;
    qcamera_logical_descriptor_t *cam = gMuxer->getLogicalCamera(device);
    CHECK_CAMERA(cam);

    // Set callbacks to HWI
    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA(pCam);

        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI(hwi);

        hwi->set_CallBacks(pCam->dev, notify_cb,data_cb, data_cb_timestamp, get_memory, user);
        // Set JPG callbacks
        // Need to revisit this to make it single callback
        if (pCam->mode == CAM_MODE_PRIMARY) {
            hwi->setJpegCallBacks(jpeg1_data_callback, user);
        }
        else {
            hwi->setJpegCallBacks(jpeg2_data_callback, user);
        }
    }
    // Store callback in Muxer to send data callbacks
    gMuxer->setDataCallback(data_cb);
    CDBG_HIGH("%s: X", __func__);

}

/*===========================================================================
 * FUNCTION   : enable_msg_type
 *
 * DESCRIPTION: Enable msg_type to send callbacks
 *
 * PARAMETERS :
 *   @device : camera hardware device info
 *   @msg_type: callback Message type to be enabled
 *
 * RETURN : None
 *==========================================================================*/
void QCameraMuxer::enable_msg_type(struct camera_device * device, int32_t msg_type)
{
    CDBG_HIGH("%s: E", __func__);
    CHECK_MUXER();
    qcamera_physical_descriptor_t *pCam = NULL;
    qcamera_logical_descriptor_t *cam = gMuxer->getLogicalCamera(device);
    CHECK_CAMERA(cam);

    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA(pCam);
        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI(hwi);
        hwi->enable_msg_type(pCam->dev, msg_type);
    }
    CDBG_HIGH("%s: X", __func__);
}

/*===========================================================================
 * FUNCTION   : disable_msg_type
 *
 * DESCRIPTION: disable msg_type to send callbacks
 *
 * PARAMETERS :
 *   @device : camera hardware device info
 *   @msg_type: callback Message type to be disabled
 *
 * RETURN : None
 *==========================================================================*/
void QCameraMuxer::disable_msg_type(struct camera_device * device, int32_t msg_type)
{
    CDBG_HIGH("%s: E", __func__);
    CHECK_MUXER();
    qcamera_physical_descriptor_t *pCam = NULL;
    qcamera_logical_descriptor_t *cam = gMuxer->getLogicalCamera(device);
    CHECK_CAMERA(cam);

    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA(pCam);
        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI(hwi);
        hwi->disable_msg_type(pCam->dev, msg_type);
    }
    CDBG_HIGH("%s: X", __func__);
}

/*===========================================================================
 * FUNCTION   : msg_type_enabled
 *
 * DESCRIPTION: Check if message type enabled
 *
 * PARAMETERS :
 *   @device : camera hardware device info
 *   @msg_type: message type
 *
 * RETURN : true/false
 *==========================================================================*/
int QCameraMuxer::msg_type_enabled(struct camera_device * device, int32_t msg_type)
{
    CDBG_HIGH("%s: E", __func__);
    CHECK_MUXER_ERROR();
    qcamera_physical_descriptor_t *pCam = NULL;
    qcamera_logical_descriptor_t *cam = gMuxer->getLogicalCamera(device);
    CHECK_CAMERA_ERROR(cam);

    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA_ERROR(pCam);

        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI_ERROR(hwi);

        if (pCam->mode == CAM_MODE_PRIMARY) {
            return hwi->msg_type_enabled(pCam->dev, msg_type);
        }
    }
    CDBG_HIGH("%s: X", __func__);
    return false;
}

/*===========================================================================
 * FUNCTION   : start_preview
 *
 * DESCRIPTION: Starts logical camera preview
 *
 * PARAMETERS :
 *   @device : camera hardware device info
 *
 * RETURN     :
 *              NO_ERROR  : success
 *              other: non-zero failure code
 *==========================================================================*/
int QCameraMuxer::start_preview(struct camera_device * device)
{
    CDBG_HIGH("%s: E", __func__);
    CHECK_MUXER_ERROR();
    int rc = NO_ERROR;
    qcamera_physical_descriptor_t *pCam = NULL;
    qcamera_logical_descriptor_t *cam = gMuxer->getLogicalCamera(device);
    CHECK_CAMERA_ERROR(cam);

    // prepare preview first for all cameras
    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA_ERROR(pCam);

        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI_ERROR(hwi);

        rc = hwi->prepare_preview(pCam->dev);
        if (rc != NO_ERROR) {
            ALOGE("%s: Error preparing preview !! ", __func__);
            return rc;
        }
    }

    if (cam->numCameras > 1) {
        // Set up sync camera sessions
        for (uint32_t i = 0; i < cam->numCameras; i++) {
            pCam = gMuxer->getPhysicalCamera(cam, i);
            CHECK_CAMERA_ERROR(pCam);

            QCamera2HardwareInterface *hwi = pCam->hwi;
            CHECK_HWI_ERROR(hwi);

            uint32_t sessionId = 0;
            if (pCam->mode == CAM_MODE_PRIMARY) {
                sessionId = cam->sId[CAM_MODE_SECONDARY];
            }
            else {
                sessionId = cam->sId[CAM_MODE_PRIMARY];
            }
            CDBG_HIGH("%s: Related cam id: %d, server id: %d sync ON"
                    "related session_id %d", __func__,
                    cam->pId[i], cam->sId[i], sessionId);
            rc = hwi->bundleRelatedCameras(true, sessionId);
            if (rc != NO_ERROR) {
                ALOGE("%s: Error Bundling physical cameras !! ", __func__);
                return rc;
            }
            // Remember Sync is ON
            cam->bSyncOn = true;
        }
    }
    // Start Preview for all cameras
    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA_ERROR(pCam);

        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI_ERROR(hwi);
        rc = hwi->start_preview(pCam->dev);
        if (rc != NO_ERROR) {
            ALOGE("%s: Error starting preview !! ", __func__);
            return rc;
        }
    }
    CDBG_HIGH("%s: X", __func__);
    return rc;
}

/*===========================================================================
 * FUNCTION   : stop_preview
 *
 * DESCRIPTION: Stops logical camera preview
 *
 * PARAMETERS :
 *   @device : camera hardware device info
 *
 * RETURN     : None
 *==========================================================================*/
void QCameraMuxer::stop_preview(struct camera_device * device)
{
    CDBG_HIGH("%s: E", __func__);
    CHECK_MUXER();
    qcamera_physical_descriptor_t *pCam = NULL;
    qcamera_logical_descriptor_t *cam = gMuxer->getLogicalCamera(device);
    CHECK_CAMERA(cam);

    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA(pCam);

        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI(hwi);

        QCamera2HardwareInterface::stop_preview(pCam->dev);
    }
    CDBG_HIGH("%s: X", __func__);
}

/*===========================================================================
 * FUNCTION   : preview_enabled
 *
 * DESCRIPTION: Checks preview enabled
 *
 * PARAMETERS :
 *   @device : camera hardware device info
 *
 * RETURN     : true/false
 *==========================================================================*/
int QCameraMuxer::preview_enabled(struct camera_device * device)
{
    CDBG_HIGH("%s: E", __func__);
    CHECK_MUXER_ERROR();
    qcamera_physical_descriptor_t *pCam = NULL;
    qcamera_logical_descriptor_t *cam = gMuxer->getLogicalCamera(device);
    CHECK_CAMERA_ERROR(cam);

    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA_ERROR(pCam);

        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI_ERROR(hwi);

        if (pCam->mode == CAM_MODE_PRIMARY) {
            return hwi->preview_enabled(pCam->dev);
        }
    }
    CDBG_HIGH("%s: X", __func__);
    return false;
}

/*===========================================================================
 * FUNCTION   : store_meta_data_in_buffers
 *
 * DESCRIPTION: Stores metadata in buffers
 *
 * PARAMETERS :
 *   @device : camera hardware device info
 *   @enable: Enable/disable metadata
 *
 * RETURN     :
 *              NO_ERROR  : success
 *              other: non-zero failure code
 *==========================================================================*/
int QCameraMuxer::store_meta_data_in_buffers(struct camera_device * device, int enable)
{
    CDBG_HIGH("%s: E", __func__);
    CHECK_MUXER_ERROR();
    int rc = NO_ERROR;
    qcamera_physical_descriptor_t *pCam = NULL;
    qcamera_logical_descriptor_t *cam = gMuxer->getLogicalCamera(device);
    CHECK_CAMERA_ERROR(cam);

    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA_ERROR(pCam);

        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI_ERROR(hwi);

        rc = hwi->store_meta_data_in_buffers(pCam->dev, enable);
        if (rc != NO_ERROR) {
            ALOGE("%s: Error storing metat data !! ", __func__);
            return rc;
        }
    }
    CDBG_HIGH("%s: X", __func__);
    return rc;
}

/*===========================================================================
 * FUNCTION   : start_recording
 *
 * DESCRIPTION: Starts recording on camcorder
 *
 * PARAMETERS :
 *   @device : camera hardware device info
 *
 * RETURN     :
 *              NO_ERROR  : success
 *              other: non-zero failure code
 *==========================================================================*/
int QCameraMuxer::start_recording(struct camera_device * device)
{
    CDBG_HIGH("%s: E", __func__);
    CHECK_MUXER_ERROR();
    int rc = NO_ERROR;
    qcamera_physical_descriptor_t *pCam = NULL;
    qcamera_logical_descriptor_t *cam = gMuxer->getLogicalCamera(device);
    CHECK_CAMERA_ERROR(cam);

    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA_ERROR(pCam);

        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI_ERROR(hwi);

        rc = hwi->start_recording(pCam->dev);
        if (rc != NO_ERROR) {
            ALOGE("%s: Error starting recording!! ", __func__);
            return rc;
        }
    }
    CDBG_HIGH("%s: X", __func__);
    return rc;
}

/*===========================================================================
 * FUNCTION   : stop_recording
 *
 * DESCRIPTION: Stops recording on camcorder
 *
 * PARAMETERS :
 *   @device : camera hardware device info
 *
 * RETURN     : None
 *==========================================================================*/
void QCameraMuxer::stop_recording(struct camera_device * device)
{
    CDBG_HIGH("%s: E", __func__);
    CHECK_MUXER();
    qcamera_physical_descriptor_t *pCam = NULL;
    qcamera_logical_descriptor_t *cam = gMuxer->getLogicalCamera(device);
    CHECK_CAMERA(cam);

    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA(pCam);

        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI(hwi);

        QCamera2HardwareInterface::stop_recording(pCam->dev);
    }
    CDBG_HIGH("%s: X", __func__);
}

/*===========================================================================
 * FUNCTION   : recording_enabled
 *
 * DESCRIPTION: Checks for recording enabled
 *
 * PARAMETERS :
 *   @device : camera hardware device info
 *
 * RETURN     : true/false
 *==========================================================================*/
int QCameraMuxer::recording_enabled(struct camera_device * device)
{
    CDBG_HIGH("%s: E", __func__);
    CHECK_MUXER_ERROR();
    qcamera_physical_descriptor_t *pCam = NULL;
    qcamera_logical_descriptor_t *cam = gMuxer->getLogicalCamera(device);
    CHECK_CAMERA_ERROR(cam);

    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA_ERROR(pCam);

        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI_ERROR(hwi);

        if (pCam->mode == CAM_MODE_PRIMARY) {
            return hwi->recording_enabled(pCam->dev);
        }
    }
    CDBG_HIGH("%s: X", __func__);
    return false;
}

/*===========================================================================
 * FUNCTION   : release_recording_frame
 *
 * DESCRIPTION: Release the recording frame
 *
 * PARAMETERS :
 *   @device : camera hardware device info
 *   @opaque: Frame to be released
 *
  * RETURN     : None
 *==========================================================================*/
void QCameraMuxer::release_recording_frame(struct camera_device * device,
                const void *opaque)
{
    CDBG_HIGH("%s: E", __func__);
    CHECK_MUXER();
    qcamera_physical_descriptor_t *pCam = NULL;
    qcamera_logical_descriptor_t *cam = gMuxer->getLogicalCamera(device);
    CHECK_CAMERA(cam);

    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA(pCam);

        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI(hwi);

        if (pCam->mode == CAM_MODE_PRIMARY) {
            QCamera2HardwareInterface::release_recording_frame(pCam->dev, opaque);
            break;
        }
    }
    CDBG_HIGH("%s: X", __func__);
}

/*===========================================================================
 * FUNCTION   : auto_focus
 *
 * DESCRIPTION: Performs auto focus on camera
 *
 * PARAMETERS :
 *   @device : camera hardware device info
 *
 * RETURN     :
 *              NO_ERROR  : success
 *              other: non-zero failure code
 *==========================================================================*/
int QCameraMuxer::auto_focus(struct camera_device * device)
{
    CDBG_HIGH("%s: E", __func__);
    CHECK_MUXER_ERROR();
    int rc = NO_ERROR;
    qcamera_physical_descriptor_t *pCam = NULL;
    qcamera_logical_descriptor_t *cam = gMuxer->getLogicalCamera(device);
    CHECK_CAMERA_ERROR(cam);

    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA_ERROR(pCam);

        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI_ERROR(hwi);
        // Call auto focus on main camera
        if (pCam->mode == CAM_MODE_PRIMARY) {
            rc = QCamera2HardwareInterface::auto_focus(pCam->dev);
            if (rc != NO_ERROR) {
                ALOGE("%s: Error auto focusing !! ", __func__);
                return rc;
            }
            break;
        }
    }
    CDBG_HIGH("%s: X", __func__);
    return rc;
}

/*===========================================================================
 * FUNCTION   : cancel_auto_focus
 *
 * DESCRIPTION: Cancels auto focus
 *
 * PARAMETERS :
 *   @device : camera hardware device info
 *
 * RETURN     :
 *              NO_ERROR  : success
 *              other: non-zero failure code
 *==========================================================================*/
int QCameraMuxer::cancel_auto_focus(struct camera_device * device)
{
    CDBG_HIGH("%s: E", __func__);
    CHECK_MUXER_ERROR();
    int rc = NO_ERROR;
    qcamera_physical_descriptor_t *pCam = NULL;
    qcamera_logical_descriptor_t *cam = gMuxer->getLogicalCamera(device);
    CHECK_CAMERA_ERROR(cam);

    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA_ERROR(pCam);

        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI_ERROR(hwi);
        // Cancel auto focus on primary camera
        if (pCam->mode == CAM_MODE_PRIMARY) {
            rc = QCamera2HardwareInterface::cancel_auto_focus(pCam->dev);
            if (rc != NO_ERROR) {
                ALOGE("%s: Error cancelling auto focus !! ", __func__);
                return rc;
            }
            break;
        }
    }
    CDBG_HIGH("%s: X", __func__);
    return rc;
}

/*===========================================================================
 * FUNCTION   : take_picture
 *
 * DESCRIPTION: Take snapshots on device
 *
 * PARAMETERS :
 *   @device : camera hardware device info
 *
 * RETURN     :
 *              NO_ERROR  : success
 *              other: non-zero failure code
 *==========================================================================*/
int QCameraMuxer::take_picture(struct camera_device * device)
{
    CDBG_HIGH("%s: E", __func__);
    CHECK_MUXER_ERROR();
    int rc = NO_ERROR;
    qcamera_physical_descriptor_t *pCam = NULL;
    qcamera_logical_descriptor_t *cam = gMuxer->getLogicalCamera(device);
    CHECK_CAMERA_ERROR(cam);

    // prepare snapshot for main camera
    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA_ERROR(pCam);

        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI_ERROR(hwi);

        if (pCam->mode == CAM_MODE_PRIMARY) {
            rc = hwi->prepare_snapshot(pCam->dev);
            if (rc != NO_ERROR) {
                ALOGE("%s: Error preparing for snapshot !! ", __func__);
                return rc;
            }
            break;
        }
    }

    // Call take picture on both cameras
    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA_ERROR(pCam);

        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI_ERROR(hwi);

        rc = QCamera2HardwareInterface::take_picture(pCam->dev);
        if (rc != NO_ERROR) {
            ALOGE("%s: Error taking picture !! ", __func__);
            return rc;
        }
    }
    CDBG_HIGH("%s: X", __func__);
    return rc;
}

/*===========================================================================
 * FUNCTION   : cancel_picture
 *
 * DESCRIPTION: Cancel the take picture call
 *
 * PARAMETERS :
 *   @device : camera hardware device info
 *
 * RETURN     :
 *              NO_ERROR  : success
 *              other: non-zero failure code
 *==========================================================================*/
int QCameraMuxer::cancel_picture(struct camera_device * device)
{
    CDBG_HIGH("%s: E", __func__);
    CHECK_MUXER_ERROR();
    int rc = NO_ERROR;
    qcamera_physical_descriptor_t *pCam = NULL;
    qcamera_logical_descriptor_t *cam = gMuxer->getLogicalCamera(device);
    CHECK_CAMERA_ERROR(cam);

    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA_ERROR(pCam);

        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI_ERROR(hwi);

        rc = QCamera2HardwareInterface::cancel_picture(pCam->dev);
        if (rc != NO_ERROR) {
            ALOGE("%s: Error cancelling picture !! ", __func__);
            return rc;
        }
    }
    CDBG_HIGH("%s: X", __func__);
    return rc;
}

/*===========================================================================
 * FUNCTION   : set_parameters
 *
 * DESCRIPTION: Sets the parameters on camera
 *
 * PARAMETERS :
 *   @device : camera hardware device info
 *   @parms : Parameters to be set on camera
 *
 * RETURN     :
 *              NO_ERROR  : success
 *              other: non-zero failure code
 *==========================================================================*/
int QCameraMuxer::set_parameters(struct camera_device * device,
        const char *parms)

{
    CDBG_HIGH("%s: E", __func__);
    CHECK_MUXER_ERROR();
    int rc = NO_ERROR;
    qcamera_physical_descriptor_t *pCam = NULL;
    qcamera_logical_descriptor_t *cam = gMuxer->getLogicalCamera(device);
    CHECK_CAMERA_ERROR(cam);

    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA_ERROR(pCam);

        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI_ERROR(hwi);

        rc = QCamera2HardwareInterface::set_parameters(pCam->dev, parms);
        if (rc != NO_ERROR) {
            ALOGE("%s: Error setting parameters !! ", __func__);
            return rc;
        }
    }
    CDBG_HIGH("%s: X", __func__);
    return rc;
}

/*===========================================================================
 * FUNCTION   : get_parameters
 *
 * DESCRIPTION: Gets the parameters on camera
 *
 * PARAMETERS :
 *   @device : camera hardware device info
 *
 * RETURN     : Parameter string or NULL
 *==========================================================================*/
char* QCameraMuxer::get_parameters(struct camera_device * device)
{
    CDBG_HIGH("%s: E", __func__);

    if (!gMuxer)
        return NULL;

    char* ret = NULL;
    int rc = NO_ERROR;
    qcamera_physical_descriptor_t *pCam = NULL;
    qcamera_logical_descriptor_t *cam = gMuxer->getLogicalCamera(device);
    if (!cam) {
        ALOGE("%s: Error getting logical camera", __func__);
        return NULL;
    }

    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        if (!pCam) {
            ALOGE("%s: Error getting physical camera", __func__);
            return NULL;
        }
        QCamera2HardwareInterface *hwi = pCam->hwi;
        if (!hwi) {
            ALOGE("%s: Allocation of hardware interface failed", __func__);
            return NULL;
        }
        if (pCam->mode == CAM_MODE_PRIMARY) {
            // Get only primary camera parameters
            ret = QCamera2HardwareInterface::get_parameters(pCam->dev);
            break;
        }
    }

    CDBG_HIGH("%s: X", __func__);
    return ret;
}

/*===========================================================================
 * FUNCTION   : put_parameters
 *
 * DESCRIPTION: Puts parameters on camera
 *
 * PARAMETERS :
 *   @device : camera hardware device info
 *   @parm : parameters
 *
 * RETURN     : None
 *==========================================================================*/
void QCameraMuxer::put_parameters(struct camera_device * device, char *parm)
{
    CDBG_HIGH("%s: E", __func__);
    CHECK_MUXER();
    qcamera_physical_descriptor_t *pCam = NULL;
    qcamera_logical_descriptor_t *cam = gMuxer->getLogicalCamera(device);
    CHECK_CAMERA(cam);

    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA(pCam);

        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI(hwi);

        if (pCam->mode == CAM_MODE_PRIMARY) {
            // Parameters are not used in HWI and hence freed
            QCamera2HardwareInterface::put_parameters(pCam->dev, parm);
            break;
        }
    }
    CDBG_HIGH("%s: X", __func__);
}

/*===========================================================================
 * FUNCTION   : send_command
 *
 * DESCRIPTION: Send command to camera
 *
 * PARAMETERS :
 *   @device : camera hardware device info
 *   @cmd : Command
 *   @arg1/arg2 : command arguments
 *
 * RETURN     :
 *              NO_ERROR  : success
 *              other: non-zero failure code
 *==========================================================================*/
int QCameraMuxer::send_command(struct camera_device * device,
        int32_t cmd, int32_t arg1, int32_t arg2)
{
    CDBG_HIGH("%s: E", __func__);
    CHECK_MUXER_ERROR();
    int rc = NO_ERROR;
    qcamera_physical_descriptor_t *pCam = NULL;
    qcamera_logical_descriptor_t *cam = gMuxer->getLogicalCamera(device);
    CHECK_CAMERA_ERROR(cam);

    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA_ERROR(pCam);

        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI_ERROR(hwi);

        rc = QCamera2HardwareInterface::send_command(pCam->dev, cmd, arg1, arg2);
        if (rc != NO_ERROR) {
            ALOGE("%s: Error sending command !! ", __func__);
            return rc;
        }
    }
    CDBG_HIGH("%s: X", __func__);
    return rc;
}

/*===========================================================================
 * FUNCTION   : release
 *
 * DESCRIPTION: Release the camera
 *
 * PARAMETERS :
 *   @device : camera hardware device info
 *
 * RETURN     : None
 *==========================================================================*/
void QCameraMuxer::release(struct camera_device * device)
{
    CDBG_HIGH("%s: E", __func__);
    CHECK_MUXER();
    qcamera_physical_descriptor_t *pCam = NULL;
    qcamera_logical_descriptor_t *cam = gMuxer->getLogicalCamera(device);
    CHECK_CAMERA(cam);

    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA(pCam);

        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI(hwi);

        QCamera2HardwareInterface::release(pCam->dev);
    }
    CDBG_HIGH("%s: X", __func__);
}

/*===========================================================================
 * FUNCTION   : dump
 *
 * DESCRIPTION: Dump the camera info
 *
 * PARAMETERS :
 *   @device : camera hardware device info
 *   @fd : fd
 *
 * RETURN     :
 *              NO_ERROR  : success
 *              other: non-zero failure code
 *==========================================================================*/
int QCameraMuxer::dump(struct camera_device * device, int fd)
{
    CDBG_HIGH("%s: E", __func__);
    CHECK_MUXER_ERROR();
    int rc = NO_ERROR;
    qcamera_physical_descriptor_t *pCam = NULL;
    qcamera_logical_descriptor_t *cam = gMuxer->getLogicalCamera(device);
    CHECK_CAMERA_ERROR(cam);

    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA_ERROR(pCam);

        QCamera2HardwareInterface *hwi = pCam->hwi;
        CHECK_HWI_ERROR(hwi);

        rc = QCamera2HardwareInterface::dump(pCam->dev, fd);
        if (rc != NO_ERROR) {
            ALOGE("%s: Error dumping", __func__);
            return rc;
        }
    }
    CDBG_HIGH("%s: X", __func__);
    return rc;
}

/*===========================================================================
 * FUNCTION   : close_camera_device
 *
 * DESCRIPTION: Close the camera
 *
 * PARAMETERS :
 *   @hw_dev : camera hardware device info
 *
 * RETURN     :
 *              NO_ERROR  : success
 *              other: non-zero failure code
 *==========================================================================*/
int QCameraMuxer::close_camera_device(hw_device_t *hw_dev)
{
    CDBG_HIGH("%s: E", __func__);
    CHECK_MUXER_ERROR();
    int rc = NO_ERROR;
    qcamera_physical_descriptor_t *pCam = NULL;
    camera_device_t *cam_dev = (camera_device_t*)hw_dev;
    qcamera_logical_descriptor_t *cam = gMuxer->getLogicalCamera(cam_dev);
    CHECK_CAMERA_ERROR(cam);

    // Unlink camera sessions
    if (cam->bSyncOn) {
        for (uint32_t i = 0; i < cam->numCameras; i++) {
            pCam = gMuxer->getPhysicalCamera(cam, i);
            CHECK_CAMERA_ERROR(pCam);

            QCamera2HardwareInterface *hwi = pCam->hwi;
            CHECK_HWI_ERROR(hwi);

            uint32_t sessionId = 0;
            if (pCam->mode == CAM_MODE_PRIMARY) {
                sessionId = cam->sId[CAM_MODE_SECONDARY];
            }
            else {
                sessionId = cam->sId[CAM_MODE_PRIMARY];
            }
            CDBG_HIGH("%s: Related cam id: %d, server id: %d sync OFF session_id %d",
                    __func__, cam->pId[i], cam->sId[i], sessionId);
            rc = hwi->bundleRelatedCameras(false, sessionId);
            if (rc != NO_ERROR) {
                ALOGE("%s: Error un-bundling cameras !! ", __func__);
            }
        }
        cam->bSyncOn = false;
    }

    for (uint32_t i = 0; i < cam->numCameras; i++) {
        pCam = gMuxer->getPhysicalCamera(cam, i);
        CHECK_CAMERA_ERROR(pCam);

        hw_device_t *dev = (hw_device_t*)(pCam->dev);
        CDBG_HIGH("%s: hw device %x, hw %x", __func__, dev, pCam->hwi);

        rc = QCamera2HardwareInterface::close_camera_device(dev);
        if (rc != NO_ERROR) {
            ALOGE("%s: Error closing camera", __func__);
            return rc;
        }
        pCam->hwi = NULL;
        pCam->dev = NULL;
    }
    // Reset JPEG client handle
    gMuxer->setJpegHandle(0);
    CDBG_HIGH("%s: X, rc: %d", __func__, rc);
    return rc;
}

/*===========================================================================
 * FUNCTION         : setupLogicalCameras
 *
 * DESCRIPTION     : Creates Camera Muxer if not created
 *
 * RETURN     :
 *              NO_ERROR  : success
 *              other: non-zero failure code
 *==========================================================================*/
int QCameraMuxer::setupLogicalCameras()
{
    int rc = NO_ERROR;
    char prop[PROPERTY_VALUE_MAX];
    int i = 0;
    camera_info info;

    CDBG_HIGH("%s[%d] E: rc = %d", __func__, __LINE__, rc);
    // Signifies whether AUX camera has to be exposed as physical camera
    property_get("persist.camera.aux.camera", prop, "0");
    bAuxCameraExposed = atoi(prop);

    // Check for number of camera present on device
    if (!m_nPhyCameras || (m_nPhyCameras > MM_CAMERA_MAX_NUM_SENSORS)) {
        ALOGE("%s: Error!! Invalid number of cameras: %d",
                __func__, m_nPhyCameras);
        return BAD_VALUE;
    }

    mPhyCamera = new qcamera_physical_descriptor_t[m_nPhyCameras];
    if (!mPhyCamera) {
        ALOGE("%s: Error allocating camera info buffer!!",__func__);
        return NO_MEMORY;
    }
    memset(mPhyCamera, 0x00,
            (m_nPhyCameras * sizeof(qcamera_physical_descriptor_t)));
    uint32_t cameraId = 0;
    m_nLogicalCameras = 0;

    // Enumerate physical cameras and logical
    for (i = 0; i < m_nPhyCameras ; i++, cameraId++) {
        camera_info *info = &mPhyCamera[i].cam_info;
        rc = QCamera2HardwareInterface::getCapabilities(cameraId,
                info, &mPhyCamera[i].type);
        mPhyCamera[i].id = cameraId;
        mPhyCamera[i].device_version = CAMERA_DEVICE_API_VERSION_1_0;
        mPhyCamera[i].mode = CAM_MODE_PRIMARY;
        if (!bAuxCameraExposed && (mPhyCamera[i].type != CAM_TYPE_MAIN)) {
            mPhyCamera[i].mode = CAM_MODE_SECONDARY;
            CDBG_HIGH("%s[%d]: Camera ID: %d, Aux Camera, type: %d",
                    __func__, __LINE__, cameraId, mPhyCamera[i].type);
        }
        else {
            m_nLogicalCameras++;
            CDBG_HIGH("%s[%d]: Camera ID: %d, Main Camera, type: %d",
                    __func__, __LINE__, cameraId, mPhyCamera[i].type);
        }
    }

    if (!m_nLogicalCameras) {
        // No Main camera detected, return from here
        return -ENODEV;
    }
    // Allocate Logical Camera descriptors
    mLogicalCamera = new qcamera_logical_descriptor_t[m_nLogicalCameras];
    if (!mLogicalCamera) {
        ALOGE("%s: Error !!!! allocating camera info buffer!!",__func__);
        delete [] mPhyCamera;
        return  NO_MEMORY;
    }
    memset(mLogicalCamera, 0x00,
            (m_nLogicalCameras * sizeof(qcamera_logical_descriptor_t)));
    // Assign MAIN cameras for each logical camera
    int index = 0;
    for (i = 0; i < m_nPhyCameras ; i++) {
        if (mPhyCamera[i].mode == CAM_MODE_PRIMARY) {
            mLogicalCamera[index].id = index;
            mLogicalCamera[index].device_version = CAMERA_DEVICE_API_VERSION_1_0;
            mLogicalCamera[index].pId[0] = i;
            mLogicalCamera[index].type[0] = CAM_TYPE_MAIN;
            mLogicalCamera[index].mode[0] = CAM_MODE_PRIMARY;
            mLogicalCamera[index].facing = mPhyCamera[i].cam_info.facing;
            mLogicalCamera[index].numCameras++;
            CDBG_HIGH("%s[%d]: Logical Main Camera ID: %d, facing: %d,"
                    "Phy Id: %d type: %d mode: %d",
                    __func__, __LINE__, mLogicalCamera[index].id,
                    mLogicalCamera[i].facing,
                    mLogicalCamera[index].pId[0],
                    mLogicalCamera[index].type[0],
                    mLogicalCamera[index].mode[0]);

            index++;
        }
    }
    //Now assign AUX cameras to logical camera
    for (i = 0; i < m_nPhyCameras ; i++) {
        if (mPhyCamera[i].mode == CAM_MODE_SECONDARY) {
            for (int j = 0; j < m_nLogicalCameras; j++) {
                int n = mLogicalCamera[j].numCameras;
                if ((n < MAX_NUM_CAMERA_PER_BUNDLE) &&
                        (mLogicalCamera[j].facing ==
                        mPhyCamera[i].cam_info.facing)) {
                    mLogicalCamera[j].pId[n] = i;
                    mLogicalCamera[j].type[n] = CAM_TYPE_AUX;
                    mLogicalCamera[j].mode[n] = CAM_MODE_SECONDARY;
                    mLogicalCamera[j].numCameras++;
                    CDBG_HIGH("%s[%d]: Logical Aux Camera ID: %d,"
                        "index: %d, aux phy id:%d, facing: %d, "
                        "Phy Id: %d type: %d mode: %d",
                        __func__, __LINE__, j, n, mLogicalCamera[j].pId[n],
                        mLogicalCamera[j].type[n], mLogicalCamera[j].mode[n]);
                }
            }
        }
    }
    //Print logical and physical camera tables
    for (i = 0; i < m_nLogicalCameras ; i++) {
        for (uint8_t j = 0; j < mLogicalCamera[i].numCameras; j++) {
            CDBG_HIGH("%s[%d]: Logical Camera ID: %d, index: %d, "
                    "facing: %d, Phy Id: %d type: %d mode: %d",
                    __func__, __LINE__, i, j, mLogicalCamera[i].facing,
                    mLogicalCamera[i].pId[j], mLogicalCamera[i].type[j],
                    mLogicalCamera[i].mode[j]);
        }
    }
    CDBG_HIGH("%s[%d] X: rc = %d", __func__, __LINE__, rc);
    return rc;
}

/*===========================================================================
 * FUNCTION   : getNumberOfCameras
 *
 * DESCRIPTION: query number of logical cameras detected
 *
 * RETURN     : number of cameras detected
 *==========================================================================*/
int QCameraMuxer::getNumberOfCameras()
{
    return m_nLogicalCameras;
}

/*===========================================================================
 * FUNCTION   : getCameraInfo
 *
 * DESCRIPTION: query camera information with its ID
 *
 * PARAMETERS :
 *   @camera_id : camera ID
 *   @info      : ptr to camera info struct
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCameraMuxer::getCameraInfo(int camera_id,
        struct camera_info *info, cam_sync_type_t *p_cam_type)
{
    int rc = NO_ERROR;
    CDBG_HIGH("%s: E, camera_id = %d", __func__, camera_id);
    cam_sync_type_t cam_type = CAM_TYPE_MAIN;

    if (!m_nLogicalCameras || (camera_id >= m_nLogicalCameras) ||
            !info || (camera_id < 0)) {
        ALOGE("%s : m_nLogicalCameras: %d, camera id: %d", __func__,
                m_nLogicalCameras, camera_id);
        return -ENODEV;
    }

    if (!mLogicalCamera || !mPhyCamera) {
        ALOGE("%s : Error! Cameras not initialized!", __func__);
        return NO_INIT;
    }
    uint32_t phy_id = mLogicalCamera[camera_id].pId[0];
    rc = QCamera2HardwareInterface::getCapabilities(phy_id, info, &cam_type);
    CDBG_HIGH("%s: X", __func__);
    return rc;
}

/*===========================================================================
 * FUNCTION   : setCallbacks
 *
 * DESCRIPTION: set callback functions to send asynchronous notifications to
 *              frameworks.
 *
 * PARAMETERS :
 *   @callbacks : callback function pointer
 *
 * RETURN     :
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCameraMuxer::setCallbacks(const camera_module_callbacks_t *callbacks)
{
    mCallbacks = callbacks;
    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : setDataCallback
 *
 * DESCRIPTION: set data callback function for snapshots
 *
 * PARAMETERS :
 *   @data_cb : callback function pointer
 *
 * RETURN     :
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCameraMuxer::setDataCallback(camera_data_callback data_cb)
{
    mDataCb = data_cb;
    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : cameraDeviceOpen
 *
 * DESCRIPTION: open a camera device with its ID
 *
 * PARAMETERS :
 *   @camera_id : camera ID
 *   @hw_device : ptr to struct storing camera hardware device info
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int QCameraMuxer::cameraDeviceOpen(int camera_id,
        struct hw_device_t **hw_device)
{
    int rc = NO_ERROR;
    uint32_t phyId = 0;
    qcamera_logical_descriptor_t *cam = NULL;

    if (camera_id < 0 || camera_id >= m_nLogicalCameras) {
        ALOGE("%s : Camera id %d not found!", __func__, camera_id);
        return -ENODEV;
    }

    if ( NULL == mLogicalCamera) {
        ALOGE("%s : Hal descriptor table is not initialized!", __func__);
        return NO_INIT;
    }
    // Get logical camera
    cam = &mLogicalCamera[camera_id];

    if (mLogicalCamera[camera_id].device_version ==
            CAMERA_DEVICE_API_VERSION_1_0) {
        // HW Dev Holders
        hw_device_t *hw_dev[cam->numCameras];

        // Open all physical cameras
        for (uint32_t i = 0; i < cam->numCameras; i++) {
            phyId = cam->pId[i];
            QCamera2HardwareInterface *hw =
                    new QCamera2HardwareInterface((uint32_t)phyId);
            if (!hw) {
                ALOGE("%s: Allocation of hardware interface failed", __func__);
                return NO_MEMORY;
            }
            if (mJpegClientHandle) {
                hw->setJpegHandleInfo(&mJpegOps, mJpegClientHandle);
            }
            hw_dev[i] = NULL;

            // Make Camera HWI aware of its mode
            cam_sync_related_sensors_event_info_t info;
            info.sync_control = CAM_SYNC_RELATED_SENSORS_ON;
            info.mode = mPhyCamera[phyId].mode;
            info.type = mPhyCamera[phyId].type;
            hw->setRelatedCamSyncInfo(&info);
            if (rc != NO_ERROR) {
                ALOGE("%s: Error sending related cam info !! ", __func__);
                return rc;
            }

            rc = hw->openCamera(&hw_dev[i]);
            if (rc != NO_ERROR) {
                delete hw;
            }
            if (mPhyCamera[phyId].type == CAM_TYPE_MAIN) {
                hw->getJpegHandleInfo(&mJpegOps, &mJpegClientHandle);
            }
            hw->getCameraSessionId(&mPhyCamera[phyId].camera_server_id);
            mPhyCamera[phyId].dev = reinterpret_cast<camera_device_t*>(hw_dev[i]);
            mPhyCamera[phyId].hwi = hw;
            cam->sId[i] = mPhyCamera[phyId].camera_server_id;
            CDBG_HIGH("%s: camera id %d server id : %d hw device %x, hw %x",
                    __func__, phyId, cam->sId[i], hw_dev[i], hw);
        }
    } else {
        ALOGE("%s: Device version for camera id %d invalid %d",
                __func__, camera_id, mLogicalCamera[camera_id].device_version);
        return BAD_VALUE;
    }

    cam->dev.common.tag = HARDWARE_DEVICE_TAG;
    cam->dev.common.version = HARDWARE_DEVICE_API_VERSION(1, 0);
    cam->dev.common.close = close_camera_device;
    cam->dev.ops = &mCameraMuxerOps;
    cam->dev.priv = (void*)cam;
    *hw_device = &cam->dev.common;
    return rc;
}


/*===========================================================================
 * FUNCTION   : getLogicalCamera
 *
 * DESCRIPTION: Get logical camera descriptor
 *
 * PARAMETERS :
 *   @device : camera hardware device info
 *
 * RETURN     : logical camera descriptor or NULL
 *==========================================================================*/
qcamera_logical_descriptor_t* QCameraMuxer::getLogicalCamera(
        struct camera_device * device)
{
    if(device && device->priv){
        return (qcamera_logical_descriptor_t*)(device->priv);
    }
    return NULL;
}

/*===========================================================================
 * FUNCTION   : getPhysicalCamera
 *
 * DESCRIPTION: Get physical camera descriptor
 *
 * PARAMETERS :
 *   @log_cam : Logical camera descriptor
 *   @index : physical camera index
 *
 * RETURN     : physical camera descriptor or NULL
 *==========================================================================*/
qcamera_physical_descriptor_t* QCameraMuxer::getPhysicalCamera(
        qcamera_logical_descriptor_t* log_cam, uint32_t index)
{
    if(!log_cam){
        return NULL;
    }
    return &mPhyCamera[log_cam->pId[index]];
}

/*===========================================================================
 * FUNCTION   : jpeg1_data_callback
 *
 * DESCRIPTION: JPEG data callback for snapshot
 *==========================================================================*/
void QCameraMuxer::jpeg1_data_callback(int32_t msg_type,
           const camera_memory_t *data, unsigned int index,
           camera_frame_metadata_t *metadata, void *user)
{
    CDBG_HIGH("%s: jpeg1 recieved", __func__);
        gMuxer->mDataCb(msg_type,
                 data,
                 index,
                 metadata,
                 user);
}

/*===========================================================================
 * FUNCTION   : jpeg2_data_callback
 *
 * DESCRIPTION: JPEG data callback for snapshot
 *==========================================================================*/
void QCameraMuxer::jpeg2_data_callback(int32_t msg_type,
           const camera_memory_t *data, unsigned int index,
           camera_frame_metadata_t *metadata, void *user)
{
    CDBG_HIGH("%s: jpeg2 recieved", __func__);
}

// Muxer Ops
camera_device_ops_t QCameraMuxer::mCameraMuxerOps = {
    set_preview_window:         QCameraMuxer::set_preview_window,
    set_callbacks:              QCameraMuxer::set_callBacks,
    enable_msg_type:            QCameraMuxer::enable_msg_type,
    disable_msg_type:           QCameraMuxer::disable_msg_type,
    msg_type_enabled:           QCameraMuxer::msg_type_enabled,

    start_preview:              QCameraMuxer::start_preview,
    stop_preview:               QCameraMuxer::stop_preview,
    preview_enabled:            QCameraMuxer::preview_enabled,
    store_meta_data_in_buffers: QCameraMuxer::store_meta_data_in_buffers,

    start_recording:            QCameraMuxer::start_recording,
    stop_recording:             QCameraMuxer::stop_recording,
    recording_enabled:          QCameraMuxer::recording_enabled,
    release_recording_frame:    QCameraMuxer::release_recording_frame,

    auto_focus:                 QCameraMuxer::auto_focus,
    cancel_auto_focus:          QCameraMuxer::cancel_auto_focus,

    take_picture:               QCameraMuxer::take_picture,
    cancel_picture:             QCameraMuxer::cancel_picture,

    set_parameters:             QCameraMuxer::set_parameters,
    get_parameters:             QCameraMuxer::get_parameters,
    put_parameters:             QCameraMuxer::put_parameters,
    send_command:               QCameraMuxer::send_command,

    release:                    QCameraMuxer::release,
    dump:                       QCameraMuxer::dump,
};


}; // namespace android
