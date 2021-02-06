
#ifndef RUNTIME_FIELD_SELECTION_HPP_
#define RUNTIME_FIELD_SELECTION_HPP_

#include "dirt_or_leaf/runtime_field_selection.h"
#include <pcl/io/pcd_io.h>

namespace pcl
{

// Retrieve field VALUE, indexed by std::string FIELD_NAME
template <typename TargetPointType, typename FieldType>
FieldType getFieldValue(TargetPointType point, std::string field_name, bool print)
{
    // Create a vector containing all the Field objects, and find the index of the target field
    std::vector<pcl::PCLPointField> fields;
    int distance_idx = pcl::getFieldIndex<TargetPointType>(field_name, fields);
    if (distance_idx == -1)
    {
        if(print)
            std::cout << "[RuntimeFieldEvaluation] Invalid point field name: " << field_name << std::endl;
        return -10e8;
    }

    // Switch from a reference to the point to an index in memory which points to it??
    const std::uint8_t* pt_data = reinterpret_cast<const std::uint8_t*> (&point);
    // Get the value in this field for this point
    FieldType field_value = 0;
    memcpy (&field_value, pt_data + fields[distance_idx].offset, sizeof (FieldType));

    return field_value;
}

template <typename TargetPointType, typename FieldType>
bool checkFieldType(std::string field_name, bool print)
{
    // Get field memory offset within point 
    std::vector<pcl::PCLPointField> fields;
    int distance_idx = pcl::getFieldIndex<TargetPointType>(field_name, fields);
    if (distance_idx == -1)
    {
        if(print)
        {
            std::cout << "[RuntimeFieldEvaluation] Invalid point field name: " << field_name << std::endl;
            std::cout << "  allowed field names for this point structure: ";
            for(int i=0; i<fields.size(); i++)
                std::cout << fields[i].name << " ";
            std::cout << std::endl;
        }
        return false;
    }
    int field_size = pcl::getFieldSize(fields[distance_idx].datatype);
    if(field_size != sizeof(FieldType))
    {
        if(print)
            std::cout << "[RuntimeFieldEvaluation] Invalid field - size is " << field_size << " instead of the expected size " << sizeof(FieldType) << std::endl;
        return false;
    }
    return true;
}

template <typename TargetPointType, typename FieldType>
bool assignValueToField(TargetPointType &point, std::string field_name, FieldType value, bool print)
{
    // Create a vector containing all the Field objects, and find the index of the target field
    std::vector<pcl::PCLPointField> fields;
    int distance_idx = pcl::getFieldIndex<TargetPointType>(field_name, fields);
    if (distance_idx == -1)
    {
        if(print)
            std::cout << "[RuntimeFieldEvaluation] Invalid point field name: " << field_name << std::endl;
        return false;
    }

    // Switch from a reference to the point to an index in memory which points to it??
    std::uint8_t* pt_data = reinterpret_cast<std::uint8_t*> (&point);
    // Get the value in this field for this point
    memcpy(pt_data + fields[distance_idx].offset, &value, sizeof (FieldType));
    return true;
}


}

#endif //RUNTIME_FIELD_SELECTION_HPP_