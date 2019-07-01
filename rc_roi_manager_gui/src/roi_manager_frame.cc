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

#include "roi_manager_frame.h"
#include "new_roi_frame.h"

#include <wx/button.h>
#include <wx/panel.h>
#include <wx/sizer.h>
#include <wx/msgdlg.h>
#include <ros/ros.h>
#include <rc_pick_client/GetRegionsOfInterest.h>
#include <rc_pick_client/SetRegionOfInterest.h>
#include <rc_pick_client/DeleteRegionsOfInterest.h>

#include "event_ids.h"
namespace rc_roi_manager_gui
{

RoiManagerFrame::RoiManagerFrame(const wxString &title)
        : wxFrame(NULL, wxID_ANY, title, wxDefaultPosition, wxSize(500, 300))
{
  nh_ = std::make_shared<ros::NodeHandle>();
  ros::Duration(1).sleep();

  if (!nh_->getParam("/rc_roi_manager_gui/pick_module", pick_module_))
  {
    ROS_ERROR("Ros parameter: /rc_roi_manager_gui/pick_module musst be set. Either rc_itempick or rc_boxpick.");
    return;
  }
  ROS_INFO_STREAM("Using: " << pick_module_ << " as pick module.");
  ROS_ASSERT((pick_module_ == "rc_itempick") || (pick_module_ == "rc_boxpick"));
  client_get_roi_ = nh_->serviceClient<rc_pick_client::GetRegionsOfInterest>
          ("/" + pick_module_ + "/get_regions_of_interest", true);
  client_delete_roi_ = nh_->serviceClient<rc_pick_client::DeleteRegionsOfInterest>(
          "/" + pick_module_ + "/delete_regions_of_interest", true);
  interactive_roi_server_ = std::make_shared<InteractiveRoiSelection>();
  client_get_roi_.waitForExistence();
  interactive_roi_server_ = std::make_shared<InteractiveRoiSelection>();

  wxPanel *panel = new wxPanel(this, -1);
  item_list_ = new wxDataViewListCtrl(panel, wxID_ANY, wxPoint(-1, -1), wxSize(-1, -1));
  item_list_->AppendTextColumn("Name", wxDATAVIEW_CELL_INERT, 200, wxALIGN_LEFT,
                               wxDATAVIEW_COL_RESIZABLE | wxDATAVIEW_COL_SORTABLE);
  item_list_->AppendTextColumn("Pose Frame", wxDATAVIEW_CELL_INERT, 120, wxALIGN_LEFT,
                               wxDATAVIEW_COL_RESIZABLE | wxDATAVIEW_COL_SORTABLE);
  item_list_->AppendTextColumn("Shape", wxDATAVIEW_CELL_INERT, 80, wxALIGN_LEFT,
                               wxDATAVIEW_COL_RESIZABLE | wxDATAVIEW_COL_SORTABLE);

  wxBoxSizer *vbox = new wxBoxSizer(wxVERTICAL);
  auto *data_box = new wxBoxSizer(wxHORIZONTAL);
  data_box->Add(item_list_, 1, wxEXPAND);
  vbox->Add(data_box, 1, wxLEFT | wxRIGHT | wxEXPAND, 10);

  auto *button_box = new wxBoxSizer(wxHORIZONTAL);
  auto *new_button = new wxButton(panel, ID_NewButton, "New");
  auto *set_button = new wxButton(panel, ID_SetButton, "Update");
  auto *edit_button = new wxButton(panel, ID_EditButton, "Edit");
  auto *delete_button = new wxButton(panel, ID_DeleteButton, "Delete");
  button_box->Add(new_button, 1);
  button_box->Add(set_button, 1);
  button_box->Add(edit_button, 1);
  button_box->Add(delete_button, 1);
  button_box->Add(-1, 0, wxEXPAND);
  vbox->Add(button_box, 0, wxTOP | wxLEFT | wxRIGHT | wxBOTTOM, 10);

  panel->SetSizer(vbox);
  updateGui();
  Centre();
  Connect(ID_NewButton, wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(RoiManagerFrame::onNewButton));
  Connect(ID_EditButton, wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(RoiManagerFrame::onEditButton));
  Connect(ID_SetButton, wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(RoiManagerFrame::onUpdateButton));
  Connect(ID_DeleteButton, wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(RoiManagerFrame::onDeleteButton));
}

void RoiManagerFrame::onDeleteButton(wxCommandEvent &)
{
  int selected_row = item_list_->GetSelectedRow();
  rc_pick_client::DeleteRegionsOfInterest srv;
  if (selected_row != wxNOT_FOUND)
  {
    srv.request.region_of_interest_ids.emplace_back(item_list_->GetTextValue(selected_row, 0));
    if (!client_delete_roi_.call(srv))
    {
      ROS_ERROR("Unable to call the service: delete_regions_of_interest");
    }
  }
  updateGui();
}

void RoiManagerFrame::onUpdateButton(wxCommandEvent &)
{
  updateGui();
}

bool RoiManagerFrame::isItemInList(wxString name)
{
  for (int i = 0; i < item_list_->GetItemCount(); i++)
  {
    auto item_name = item_list_->GetTextValue(i, 0);
    if (wxString(name) == item_name) return true;
  }
  return false;
}

void RoiManagerFrame::onEditButton(wxCommandEvent &)
{
  int selected_row = item_list_->GetSelectedRow();
  if (selected_row != wxNOT_FOUND)
  {
    rc_pick_client::GetRegionsOfInterest srv;
    srv.request.region_of_interest_ids.emplace_back(item_list_->GetTextValue(selected_row, 0));
    if (client_get_roi_.call(srv))
    {
      create_roi_frame_ = new NewRoiFrame("New Roi Frame", this, pick_module_);
      create_roi_frame_->setRoi(srv.response.regions_of_interest[0], true);
      create_roi_frame_->Show(true);
    }
    else
    {
      ROS_ERROR("Failed to call service: get_regions_of_interest");
    }
  }
}

void RoiManagerFrame::onNewButton(wxCommandEvent &)
{
  create_roi_frame_ = new NewRoiFrame("New Roi Frame", this, pick_module_);
  rc_pick_client::RegionOfInterest roi;
  roi.id = "";
  roi.primitive.dimensions = {0.1, 0.1, 0.1};
  roi.pose.pose.orientation.w = 1;
  roi.pose.header.frame_id = "camera";
  roi.primitive.type = 1;
  create_roi_frame_->setRoi(roi, false);
  create_roi_frame_->Show(true);
}

void RoiManagerFrame::updateGui()
{
  item_list_->DeleteAllItems();
  rc_pick_client::GetRegionsOfInterest srv;
  wxString shapes[] = {"Box", "Sphere"};
  ros::Duration(0.01).sleep();
  if (client_get_roi_.call(srv))
  {
    ROS_DEBUG("Size of vector of regions of interest: %zu", srv.response.regions_of_interest.size());
    for (rc_pick_client::RegionOfInterest roi : srv.response.regions_of_interest)
    {
      wxVector <wxVariant> item;
      wxString pose_frame = wxString(roi.pose.header.frame_id);
      wxString name = wxString(roi.id);
      wxString shape = shapes[roi.primitive.type - 1];
      item.push_back(name);
      item.push_back(pose_frame);
      item.push_back(shape);
      item_list_->AppendItem(item);
    }
  }
  else
  {
    ROS_ERROR("UpdateGui: Failed to call service get_regions_of_interest");
  }
}
} //rc_roi_manager_gui
