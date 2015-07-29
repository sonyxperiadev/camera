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
 *
 */

#ifndef __QCAMERAPERF_H__
#define __QCAMERAPERF_H__

#include <dlfcn.h>
#include <utils/Mutex.h>

#define ONE_SEC 1000

typedef enum {
    ALL_CPUS_PWR_CLPS_DIS = 0x101,
    CPU0_MIN_FREQ_TURBO_MAX = 0x2FE,
    CPU4_MIN_FREQ_TURBO_MAX = 0x1FFE,
}perf_lock_params_t;

using namespace android;

namespace qcamera {

class QCameraPerfLock {
public:
    QCameraPerfLock();
    ~QCameraPerfLock();

    void    lock_init();
    void    lock_deinit();
    int32_t lock_rel();
    int32_t lock_acq();
private:
    int32_t        (*perf_lock_acq)(int, int, int[], int);
    int32_t        (*perf_lock_rel)(int);
    void           *mDlHandle;
    uint32_t        mPerfLockEnable;
    Mutex           mLock;
    int32_t         mPerfLockHandle;  // Performance lock library handle
};

}; // namespace qcamera

#endif /* __QCAMREAPERF_H__ */
