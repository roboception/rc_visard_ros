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

#include "new_roi_frame.h"
#include <wx/button.h>
#include <wx/panel.h>
#include <wx/sizer.h>
#include <geometry_msgs/Pose.h>
#include <rc_pick_client/SetRegionOfInterest.h>

#include "event_ids.h"

namespace rc_roi_manager_gui
{


NewRoiFrame::NewRoiFrame(const wxString &title, RoiManagerFrame *manager, std::string pick_module)
        : wxFrame(NULL, wxID_ANY, title, wxDefaultPosition, wxSize(300, 280)), manager_(manager)
{
  manager_->Show(false);
  nh_ = std::make_shared<ros::NodeHandle>();
  client_set_roi_ = nh_->serviceClient<rc_pick_client::SetRegionOfInterest>(
          "/" + pick_module + "/set_region_of_interest");

  wxPanel *panel = new wxPanel(this, -1);
  auto name_str = new wxStaticText(panel, wxID_ANY, wxT("Roi Name"));
  name_box_ = new wxTextCtrl(panel, wxID_ANY);

  auto pose_frame_str = new wxStaticText(panel, wxID_ANY, wxT("Pose Frame"));
  wxString pose_frame_choices[] = {"camera", "external"};
  pose_frame_box_ = new wxChoice(panel, wxID_ANY, wxDefaultPosition, wxDefaultSize, 2, pose_frame_choices);

  auto shape_str = new wxStaticText(panel, wxID_ANY, wxT("Shape"));
  wxString shape_choices[] = {"box", "sphere"};
  shape_box_ = new wxChoice(panel, wxID_ANY, wxDefaultPosition, wxDefaultSize, 2, shape_choices);

  auto *vbox = new wxBoxSizer(wxVERTICAL);
  vbox->Add(name_str, 0, wxLEFT | wxRIGHT | wxTOP, 10);
  vbox->Add(name_box_, 0, wxEXPAND | wxLEFT | wxRIGHT | wxTOP, 10);
  vbox->Add(pose_frame_str, 0, wxLEFT | wxTOP, 10);
  vbox->Add(pose_frame_box_, 0, wxLEFT | wxTOP, 10);
  vbox->Add(shape_str, 0, wxLEFT | wxTOP, 10);
  vbox->Add(shape_box_, 0, wxLEFT | wxTOP, 10);

  wxButton *btn1 = new wxButton(panel, ID_SaveButton, wxT("Save"));
  wxButton *btn2 = new wxButton(panel, ID_UpdateRoiButton, wxT("Update"));
  wxButton *btn3 = new wxButton(panel, ID_CancelButton, wxT("Cancel"));
  auto *hbox = new wxBoxSizer(wxHORIZONTAL);
  hbox->Add(btn1, 0, wxLEFT | wxTOP, 10);
  hbox->Add(btn2, 0, wxLEFT | wxTOP, 10);
  hbox->Add(btn3, 0, wxLEFT | wxTOP, 10);
  vbox->Add(hbox);
  panel->SetSizer(vbox);

  CenterOnParent();
  Connect(ID_SaveButton, wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(NewRoiFrame::onSaveButton));
  Connect(ID_CancelButton, wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(NewRoiFrame::onCancelButton));
  Connect(ID_UpdateRoiButton, wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(NewRoiFrame::onUpdateButton));
}

void NewRoiFrame::setRoi(rc_pick_client::RegionOfInterest roi, bool edit)
{
  is_editing_ = edit;
  roi_ = roi;
  name_box_->AppendText(roi_.id);
  shape_box_->SetSelection(roi_.primitive.type - 1);
  pose_frame_box_->SetStringSelection(roi_.pose.header.frame_id);
  if (!manager_->interactive_roi_server_->setInteractiveRoi(roi_))
  {
    ROS_ERROR("Failed to set the interactive region of interest");
  }
}

//TODO get suggests that you are getting something in return. What is this function doing?
void NewRoiFrame::setFromInteractiveRoi()
{
  if (!manager_->interactive_roi_server_->getInteractiveRoi(roi_))
  {
    ROS_ERROR("Failed to get the interactive region of interest");
  }
}


void NewRoiFrame::onCancelButton(wxCommandEvent &)
{
  this->Close();
}

void NewRoiFrame::onSaveButton(wxCommandEvent &)
{
  setFromInteractiveRoi();

  if (manager_->isItemInList(name_box_->GetValue()) && (roi_.id != name_box_->GetValue()))
  {
    wxMessageDialog *pop_up = new wxMessageDialog(this, "A region of interest with that name already exists!",
                                                  "Name Error");
    pop_up->ShowModal();
    return;
  }
  if (is_editing_)
  {
    wxCommandEvent evt = wxCommandEvent(wxEVT_COMMAND_BUTTON_CLICKED, ID_DeleteButton);
    manager_->onDeleteButton(evt);
  }

  roi_.id = name_box_->GetValue();
  roi_.pose.header.frame_id = pose_frame_box_->GetStringSelection();
  roi_.primitive.type = shape_box_->GetSelection() + 1;
  rc_pick_client::SetRegionOfInterest srv;
  srv.request.region_of_interest = roi_;
  if (!client_set_roi_.call(srv))
  {
    ROS_ERROR("Failed to call service set_regions_of_interest");
  }
  this->Close();
}

void NewRoiFrame::onUpdateButton(wxCommandEvent &)
{
  setFromInteractiveRoi();
  roi_.pose.header.frame_id = pose_frame_box_->GetStringSelection();
  roi_.primitive.type = shape_box_->GetSelection() + 1;
  if (!manager_->interactive_roi_server_->setInteractiveRoi(roi_))
  {
    ROS_ERROR("Failed to set the interactive region of interest");
  }

}

NewRoiFrame::~NewRoiFrame()
{
  manager_->updateGui();
  setFromInteractiveRoi();
  manager_->Show(true);
}


} //rc_roi_manager_gui
