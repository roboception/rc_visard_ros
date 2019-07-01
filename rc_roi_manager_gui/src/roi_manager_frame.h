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

#ifndef _ROIMANAGERFRAME_H_
#define _ROIMANAGERFRAME_H_

#include <wx/frame.h>
#include <wx/wx.h>
#include <wx/dataview.h>
#include <ros/ros.h>
#include "interactive_roi_selection.h"

namespace rc_roi_manager_gui
{

class NewRoiFrame;

class RoiManagerFrame : public wxFrame
{
  public:

    std::shared_ptr<InteractiveRoiSelection> interactive_roi_server_;

    /**
     * @brief Constructor.
     * @param title title of the window
     */
    explicit RoiManagerFrame(const wxString &title);

    virtual ~RoiManagerFrame() = default;

    /**
     * @brief Update list of shown rois on the gui
     */
    void updateGui();

    /**
     * @brief Search if an roi is already present
     * @param name Name of the roi to be searched for.
     * @return Returns true if the roi is found in the item_list_
     */
    bool isItemInList(wxString name);

    /**
     * @brief Event handler for the delete button
     */
    void onDeleteButton(wxCommandEvent &);

  private:

    wxDataViewListCtrl *item_list_;
    NewRoiFrame *create_roi_frame_;
    std::shared_ptr<ros::NodeHandle> nh_;
    ros::ServiceClient client_get_roi_;
    ros::ServiceClient client_delete_roi_;
    std::string pick_module_;

    /**
     * @brief Event handler for the update button
     */
    void onUpdateButton(wxCommandEvent &);

    /**
     * @brief Event handler for the edit button
     */
    void onEditButton(wxCommandEvent &);

    /**
     * @brief Event handler for the new button
     */
    void onNewButton(wxCommandEvent &);

};


} //rc_roi_manager_gui

#endif
