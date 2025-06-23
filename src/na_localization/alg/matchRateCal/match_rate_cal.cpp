#include "match_rate_cal.h"

namespace alg{

    MatchRateCal::MatchRateCal(){
        ivox_options_.resolution_ = 0.5;
        ivox_options_.capacity_ = 1e9;
        ivox_map_ = std::make_shared<IVoxType>(ivox_options_);
    }

    bool MatchRateCal::setMap(const utils::PointICloudPtr& map_ptr){
        ivox_map_->AddPoints(map_ptr->points);
        if(map_ptr->size() > 0){
            return true;
        }else{
            return false;
        }
        
    }

    bool MatchRateCal::calculateMatchRate(const utils::PointICloudPtr& aft_map_cloud_ptr_,
                                double& match_rate ){
        int eligibleNums{0};
        // cout << "aft_map_cloud_ptr_: " << aft_map_cloud_ptr_->size() << endl;
        for(int i=0; i<aft_map_cloud_ptr_->size(); ++i){
            // cout << i << " ";
            if(ivox_map_->JudgeClosetPoint(aft_map_cloud_ptr_->points[i])){
                ++eligibleNums;
            }
        } 
        match_rate = eligibleNums/double(aft_map_cloud_ptr_->size());

        if(match_rate > match_rate_threshold_){
            return true;
        }else{
            return false;
        }

        return false;
    }




}