#ifndef INSMECH_HPP
#define INSMECH_HPP

#include "Global_defs.hpp"
#include "Earth.hpp"
#include "Rotation.hpp"


class INSMech
{
public:
    // mechanism
    static void insMech(InsState &InsStateCur, InsState &InsStatePrev, 
                        const ImuData &ImuCur, const ImuData &ImuPrev);
    

private:
    // Velocity integration
    static void VelInterg(InsState &InsStateCur, InsState &InsStatePrev, 
                        const ImuData &ImuCur, const ImuData &ImuPrev);

    // Position integration
    static void PosInterg(InsState &InsStateCur, InsState &InsStatePrev, 
                        const ImuData &ImuCur, const ImuData &ImuPrev);                        

    // Attitude integration
    static void AttInterg(InsState &InsStateCur, InsState &InsStatePrev, 
                        const ImuData &ImuCur, const ImuData &ImuPrev);    
};












#endif // INSMECH_HPP