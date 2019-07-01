/*
 * Copyright (c) 2019 Roboception GmbH
 *
 * Author: Carlos Xavier Garcia Briones
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef NEWROIFRAME_H
#define NEWROIFRAME_H

#include <wx/frame.h>
#include <wx/statline.h>
#include <wx/wx.h>
#include <vector>
#include <memory>
#include "roi_manager_frame.h"

namespace rc_roi_manager_gui
{

class RoiManagerFrame;

class NewRoiFrame : public wxFrame
{

  public:
    /**
     * @brief Constructor
     * @param title Title of the wxFrame
     * @param manager Pointer to the main RoiManagerFrame
     * @param pick_module Name of the module being used. Either rc_itempick or rc_boxpick.
     */
    NewRoiFrame(const wxString &title, RoiManagerFrame *manager, std::string pick_module);

    virtual ~NewRoiFrame();

    /**
     * @brief sets the member roi_
     * @param roi Region of Interest
     * @param edit If true a roi is being editted. Otherwise a new roi is being created.
     */
    void setRoi(rc_pick_client::RegionOfInterest roi, bool edit);

  private:

    /**
     * @brief Ros service that sets the region of interest from roi in interactive_roi_selection server
     */
    void setFromInteractiveRoi();

    /**
     * @brief Event handler for the save button
     */
    void onSaveButton(wxCommandEvent &);

    /**
     * @brief Event handler for the cancel button
     */
    void onCancelButton(wxCommandEvent &);

    /**
     * @brief Event handler for the update button
     */
    void onUpdateButton(wxCommandEvent &);

  private:
    wxTextCtrl *name_box_;
    wxChoice *shape_box_;
    wxChoice *pose_frame_box_;
    rc_pick_client::RegionOfInterest roi_;
    std::shared_ptr <ros::NodeHandle> nh_;
    ros::ServiceClient client_set_roi_;
    RoiManagerFrame *manager_;
    bool is_editing_;


};
} //rc_roi_manager_gui

#endif //NEWROIFRAME_H
