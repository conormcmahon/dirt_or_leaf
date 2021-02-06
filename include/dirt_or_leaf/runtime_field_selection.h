
#ifndef RUNTIME_FIELD_SELECTION_
#define RUNTIME_FIELD_SELECTION_

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

namespace pcl
{
    // Handling Point Fields at Runtime
    //   Get value at selected field for a given point
    template <typename TargetPointType, typename FieldType>
    FieldType getFieldValue(TargetPointType point, std::string field_name, bool print=true);
    //   Ensure Field Exists and Is Float
    //      currently, this only checks field SIZE, should be possible to check based on field information in pcl::PCLPointField struct? 
    template <typename TargetPointType, typename FieldType>
    bool checkFieldType(std::string field_name, bool print=true);
    //   Similarly, assign a value to a selected field for a given point
    template <typename TargetPointType, typename FieldType>
    bool assignValueToField(TargetPointType &point, std::string field_name, FieldType value, bool print=true);
}

#endif //RUNTIME_FIELD_SELECTION_