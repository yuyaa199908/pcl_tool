// #include <pcl/memory.h>
// #include <pcl/pcl_base.h>
// #include <pcl/pcl_macros.h>
// #include <pcl/search/search.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>

#ifndef PCL_REGION_GROWING_BALADO_H_
#define PCL_REGION_GROWING_BALADO_H_

#include <pcl/segmentation/region_growing.h>


namespace pcl
{
    template <typename PointT, typename NormalT>
    class PCL_EXPORTS RegionGrowingBalado : public RegionGrowing<PointT, NormalT> 
    {
        // public:           
        public:
            // using KdTree = pcl::search::Search<PointT>;
            // using KdTreePtr = typename KdTree::Ptr;
            // using Normal = pcl::PointCloud<NormalT>;
            // using NormalPtr = typename Normal::Ptr;
            // using PointCloud = pcl::PointCloud<PointT>;
        
            // using PCLBase <PointT>::input_;
            // using PCLBase <PointT>::indices_;
            // using PCLBase <PointT>::initCompute;
            // using PCLBase <PointT>::deinitCompute;

            double neighbour_distance_ {0.05};
            unsigned int neighbour_number_ {50};

            // KdTreePtr search_{nullptr};
            void setDistanceOfNeighbours (double neighbour_distance){
                neighbour_distance_ = neighbour_distance;
            }

            double getDistanceOfNeighbours(){
                return (neighbour_distance_);
            }

        protected:
            void findPointNeighbours (){
                pcl::Indices neighbours;
                std::vector<float> distances;
                this->point_neighbours_.resize (this->input_->size (), neighbours);
                if (this->input_->is_dense)
                {
                    for (const auto& point_index: (*this->indices_))
                    {
                        neighbours.clear ();
                        // search_->nearestKSearch (point_index, neighbour_number_, neighbours, distances);
                        this->search_->radiusSearch (point_index, neighbour_distance_, neighbours, distances,neighbour_number_);  //error
                        this->point_neighbours_[point_index].swap (neighbours);
                    }
                }
                else
                {
                    std::cout << "input_->is_dense is False"<< std::endl;
                    for (const auto& point_index: (*this->indices_))
                    {
                        if (!pcl::isFinite ((*this->input_)[point_index])) continue;
                        neighbours.clear ();
                        // search_->nearestKSearch (point_index, neighbour_number_, neighbours, distances);
                        this->search_->radiusSearch (point_index, neighbour_distance_, neighbours, distances,neighbour_number_);
                        this->point_neighbours_[point_index].swap (neighbours);
                    }
                }
            }
            
    };
}

#endif