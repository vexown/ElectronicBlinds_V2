/* MMC56x3_wrapper.cpp - Implementation of the C wrapper for MMC56x3 */

#include "MMC56x3_wrapper.h"  // C header with compatible declarations
#include "MMC56x3.hpp"        // C++ header that we're wrapping

/* The "opaque pointer" pattern - This is the key to C/C++ interoperability
   In C code, users only see this as an incomplete type they can't access directly
   In C++ code, we know the full implementation details */
struct MMC56x3_Instance 
{
    MMC56x3* instance;  // Pointer to the actual C++ class instance
};

/* "Constructor" function - Creates and returns an opaque handle to a C++ object
   This is the C-compatible replacement for "new MMC56x3()" */
MMC56x3_Instance* MMC56x3_Create(i2c_inst_t* i2c_instance, uint8_t address) 
{
    /* Use C++ memory allocation (new) since we're working with C++ objects */
    MMC56x3_Instance* wrapper = new MMC56x3_Instance;  // Allocate the wrapper
    wrapper->instance = new MMC56x3(i2c_instance, address);  // Create the C++ object
    return wrapper;  // Return opaque pointer to C code
}

/* "Destructor" function - Cleans up memory for both wrapper and C++ object
   This is the C-compatible replacement for "delete" */
void MMC56x3_Destroy(MMC56x3_Instance* wrapper) 
{
    if (wrapper)  // Safety check - important in C interfaces
    {
        delete wrapper->instance;  // Delete the C++ object first
        delete wrapper;            // Then delete the wrapper
    }
}

/* Each wrapper function follows this pattern:
   1. Check if the pointer is valid
   2. Call the corresponding C++ method
   3. Handle any data conversion between C and C++ types
   4. Return results in a C-compatible way */
bool MMC56x3_Begin(MMC56x3_Instance* wrapper) 
{
    if (wrapper && wrapper->instance)  // Null check is critical for C interfaces
    {
        return wrapper->instance->begin();  // Forward the call to C++ method
    }
    return false;  // Safe default return value 
}

void MMC56x3_Reset(MMC56x3_Instance* wrapper) 
{
    if (wrapper && wrapper->instance) 
    {
        wrapper->instance->reset();  // Void methods just need the forwarding
    }
    // No return for void functions
}

void MMC56x3_SetDataRate(MMC56x3_Instance* wrapper, uint8_t rate) 
{
    if (wrapper && wrapper->instance) 
    {
        wrapper->instance->setDataRate(rate);  // Simply pass the parameter through
    }
}

/* This function shows data structure translation between C++ and C
   It handles converting from the C++ class's MagData to our C-compatible struct */
bool MMC56x3_ReadData(MMC56x3_Instance* wrapper, MMC56x3_MagData* data) 
{
    if (wrapper && wrapper->instance) 
    {
        MMC56x3::MagData cppData;  // Create a C++ structure to hold data temporarily
        bool result = wrapper->instance->readData(cppData);  // Call C++ method
        if (result) 
        {
            // Copy data from C++ struct to C struct field by field
            data->x = cppData.x;
            data->y = cppData.y;
            data->z = cppData.z;
            data->temperature = cppData.temperature;
        }
        return result;  // Return the success/failure result
    }
    return false;
}

/* Additional wrapper functions follow the same pattern */

float MMC56x3_ReadTemperature(MMC56x3_Instance* wrapper) 
{
    if (wrapper && wrapper->instance) 
    {
        return wrapper->instance->readTemperature();  // Direct value return
    }
    return 0.0f; // Return a sensible default for error case
}

void MMC56x3_MagnetSetReset(MMC56x3_Instance* wrapper) 
{
    if (wrapper && wrapper->instance) 
    {
        wrapper->instance->magnetSetReset();
    }
}

void MMC56x3_SetContinuousMode(MMC56x3_Instance* wrapper, bool mode) 
{
    if (wrapper && wrapper->instance) 
    {
        wrapper->instance->setContinuousMode(mode);  // C++ can use bool directly
    }
}

bool MMC56x3_IsContinuousMode(MMC56x3_Instance* wrapper) 
{
    if (wrapper && wrapper->instance) 
    {
        return wrapper->instance->isContinuousMode();
    }
    return false;  // Default return for error case
}