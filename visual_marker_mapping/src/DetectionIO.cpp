#include "visual_marker_mapping/DetectionIO.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace camSurv
{
//-----------------------------------------------------------------------------
DetectionResult readDetectionResult(const std::string& filename)
{
	boost::property_tree::ptree ptree;
    boost::property_tree::read_json(filename, ptree);

    DetectionResult result;
    for(const auto& pt : ptree.get_child("images"))
    {
        TagImg img;
        img.imageId = pt.second.get<int>("id");
        img.filename = pt.second.get<std::string>("filename");;
        result.images.push_back(img);
    }

    for (const auto& pt : ptree.get_child("tags"))
    {
        Tag tag;
        tag.tagId = pt.second.get<int>("id");
        tag.tagType = pt.second.get<string>("tag_type");
        tag.width = pt.second.get<double>("width");
        tag.height = pt.second.get<double>("height");
        result.tags.push_back(tag);
    }


    for(const auto& pt : ptree.get_child("tag_observations"))
    {
    	TagObservation tagObs;
        for (const auto& corner : pt.second.get_child("observations"))
        {
            std::vector<double> values;
            for (const auto& value : corner.second)
                values.push_back(value.second.get_value<double>());

            if (values.size()!=2)
                throw std::runtime_error("Unexpected number of values");

            tagObs.corners.emplace_back(values.at(0), values.at(1));
        }
        if (tagObs.corners.size()!=4)
            throw std::runtime_error("Unexpected number of values");
        
        tagObs.imageId = pt.second.get<int>("image_id");
        tagObs.tagId = pt.second.get<int>("tag_id");
        result.tagObservations.push_back(tagObs);
    }
    
    return result;
}
//-----------------------------------------------------------------------------
bool writeDetectionResult(const DetectionResult& result, const std::string& filename)
{
	boost::property_tree::ptree ptree;

	boost::property_tree::ptree pt_imgs;
	for (const auto& img : result.images)
	{
		boost::property_tree::ptree pt;
        pt.put("filename", img.filename);
        pt.put("id", img.imageId);
        pt_imgs.push_back(std::make_pair("", pt));
    }
    ptree.add_child("images", pt_imgs);

    boost::property_tree::ptree pt_tags;
	for (const auto& tag : result.tags)
	{
		boost::property_tree::ptree pt;
        pt.put("id", tag.tagId);
        pt.put("tag_type", tag.tagType);
        pt.put("width", tag.width);
        pt.put("height", tag.height);
        pt_tags.push_back(std::make_pair("", pt));
    }
    ptree.add_child("tags", pt_tags);

    boost::property_tree::ptree pt_tag_observations;
	for (const auto& tagObs : result.tagObservations)
	{
		boost::property_tree::ptree pt;
        pt.put("image_id", tagObs.imageId);
        pt.put("tag_id", tagObs.tagId);

        auto mkPt = []( double p ) 
        {
            boost::property_tree::ptree pt;
            pt.put("", p); 
            return pt;
        };
        boost::property_tree::ptree observations;
        for (int i = 0; i < 4; ++i) 
        {
            boost::property_tree::ptree coord;
            coord.push_back(std::make_pair("", mkPt(tagObs.corners[i].x())));
            coord.push_back(std::make_pair("", mkPt(tagObs.corners[i].y())));
            observations.push_back(std::make_pair("", coord));
        }

	    pt.add_child("observations", observations);
        pt_tag_observations.push_back(std::make_pair("", pt));
    }
    ptree.add_child("tag_observations", pt_tag_observations);

    boost::property_tree::json_parser::write_json(filename, ptree);

    return true;

}
//-----------------------------------------------------------------------------
}
