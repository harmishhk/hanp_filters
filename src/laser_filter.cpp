/*/
 * Copyright (c) 2015 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution and use  in source  and binary  forms,  with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
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
 *
 *                                  Harmish Khambhaita on Wed Jul 22 2015
 */

// parameters configurable before starting
#define HUMAN_RADIUS 0.25 // meters
#define TRACKED_HUMANS_TOPIC "humans"
#define DEFAULT_HUMAN_SEGMENT hanp_msgs::TrackedSegmentType::TORSO

// other parameters
#define SUBSCRIBERS_QUEUE_SIZE 5
#define PUBLISHERS_QUEUE_SIZE 5

#include <math.h>

#include <hanp_filters/laser_filter.h>
#include <hanp_msgs/TrackedSegmentType.h>

// declare the LaserFilter as a pluginlib class
#include "pluginlib/class_list_macros.h"
PLUGINLIB_EXPORT_CLASS(hanp_filters::LaserFilter, filters::FilterBase<sensor_msgs::LaserScan>)

namespace hanp_filters
{
    // empty constructor and destructor
    LaserFilter::LaserFilter() {}
    LaserFilter::~LaserFilter() {}

    bool LaserFilter::configure()
    {
        // get private node handle
        ros::NodeHandle private_nh("~/");

        // get parameters from parameter server
        if(!getParam("human_radius", human_radius_))
        {
            human_radius_ = HUMAN_RADIUS;
            ROS_WARN("no human_radius specified, using defalut value of %f meters", HUMAN_RADIUS);
        }
        ROS_DEBUG("using human_radius of %f meters", human_radius_);

        // set-up subscribers and publishers
        std::string tracked_humans_topic;
        private_nh.param("tracked_humans_topic", tracked_humans_topic, std::string(TRACKED_HUMANS_TOPIC));
        tracked_humans_sub_ = private_nh.subscribe(tracked_humans_topic, SUBSCRIBERS_QUEUE_SIZE, &LaserFilter::trackedHumansReceived, this);

        private_nh.param("default_human_segment", default_human_segment_, (int)(DEFAULT_HUMAN_SEGMENT));

        // listen tf for some time to aviod exploration-in-past errors
        ros::Duration(0.5).sleep();
    }

    bool LaserFilter::update(const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out)
    {
        // get tracked humans pointer locally for thread safety
        auto tracked_humans = last_tracked_humans_;

        if(tracked_humans.humans.size() > 0)
        {
            // transform humans to scan frame
            int res;
            hanp_msgs::TrackedHumans tracked_humans_transformed;
            try
            {
                std::string error_msg;
                res = tf_.waitForTransform(scan_in.header.frame_id, tracked_humans.header.frame_id,
                    ros::Time(0), ros::Duration(0.5), ros::Duration(0.01), &error_msg);
                tf::StampedTransform humans_to_scan_transform;
                tf_.lookupTransform(scan_in.header.frame_id, tracked_humans.header.frame_id,
                    ros::Time(0), humans_to_scan_transform);

                for(auto tracked_human : tracked_humans.humans)
                {
                    // check if default segment exists for this human
                    auto segment_it = tracked_human.segments.end();
                    for(auto it = tracked_human.segments.begin(); it != tracked_human.segments.end(); ++it)
                    {
                        if(it->type == default_human_segment_)
                        {
                            segment_it = it;
                            break;
                        }
                    }

                    // don't consider human if it does not have default segment
                    if(segment_it == tracked_human.segments.end())
                    {
                        continue;
                    }
                    else
                    {
                        auto segment = *segment_it;

                        hanp_msgs::TrackedSegment tracked_human_segment_transformed;
                        tf::Pose human_segment_in;
                        tf::poseMsgToTF(segment.pose.pose, human_segment_in);
                        tf::poseTFToMsg(humans_to_scan_transform * human_segment_in, tracked_human_segment_transformed.pose.pose);

                        hanp_msgs::TrackedHuman tracked_human_transformed;
                        tracked_human_transformed.track_id = tracked_human.track_id;
                        tracked_human_transformed.segments.push_back(tracked_human_segment_transformed);

                        tracked_humans_transformed.humans.push_back(tracked_human_transformed);
                    }
                }
                tracked_humans_transformed.header.stamp = humans_to_scan_transform.stamp_;
                tracked_humans_transformed.header.frame_id = humans_to_scan_transform.frame_id_;
            }
            catch(const tf::TransformException &ex)
            {
                ROS_ERROR("transform failure (%d): %s", res, ex.what());
            }

            return filterScan(scan_in, scan_out, tracked_humans_transformed, human_radius_);
        }
        else
        {
            scan_out = scan_in;
            return true;
        }
    }

    void LaserFilter::trackedHumansReceived(const hanp_msgs::TrackedHumans& tracked_humans)
    {
        last_tracked_humans_ = tracked_humans;
    }

    bool LaserFilter::filterScan(const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out, hanp_msgs::TrackedHumans& humans, double human_radius)
    {
        scan_out = scan_in;

        // collect human segments responsible for filtring scans
        std::vector<hanp_msgs::TrackedSegment> human_segments;
        for(auto human : humans.humans)
        {
            // check if default segment exists for this human
            auto segment_it = human.segments.end();
            for(auto it = human.segments.begin(); it != human.segments.end(); ++it)
            {
                if(it->type == default_human_segment_)
                {
                    segment_it = it;
                    break;
                }
            }

            // don't consider human if it does not have default segment
            if(segment_it == human.segments.end())
            {
                continue;
            }
            else
            {
                human_segments.push_back(*segment_it);
            }
        }

        // now filter the scans
        for(auto i = 0; i < scan_in.ranges.size(); i++)
        {
            auto range = scan_in.ranges[i];
            if((!std::isnan(range)) && (range < scan_in.range_max) && (range > scan_in.range_min))
            {
                // get the position of the range point in scan frame
                auto angle = scan_in.angle_min + (i * scan_in.angle_increment);
                auto rangex = range * cos(angle);
                auto rangey = range * sin(angle);

                // if range value is due to human, change it to range_max
                for(auto segment : human_segments)
                {
                    auto dist = hypot(rangex - segment.pose.pose.position.x, rangey - segment.pose.pose.position.y);
                    if (dist < human_radius)
                    {
                        scan_out.ranges[i] = scan_out.range_max;
                    }
                }
            }
        }

        return true;
    }
}
