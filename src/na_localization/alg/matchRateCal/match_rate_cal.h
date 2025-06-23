#ifndef MATCH_RATE_CAL_HPP_
#define MATCH_RATE_CAL_HPP_

#include "alg/common/common.h"

namespace alg{

class MatchRateCal{
    using IVoxType = faster_lio::IVox<3, faster_lio::IVoxNodeType::DEFAULT, utils::PointI>;

    public:
        MatchRateCal();

        bool setMap(const utils::PointICloudPtr& map_ptr);

        bool calculateMatchRate(const utils::PointICloudPtr& aft_map_cloud_ptr_,
                                double& match_rate );

    private:
        IVoxType::Options ivox_options_;
        std::shared_ptr<IVoxType> ivox_map_;
        double match_rate_threshold_{0.75};                             
    
};


}



#endif