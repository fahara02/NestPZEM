#ifndef SYSTEM_EVENTS_HPP
#define SYSTEM_EVENTS_HPP
namespace pzemCore
{
enum class Event {
    NEW_DEVICE,
    UPDATE,
    STALL,
    READING_IR,
    READING_HR,
    FULL_IR_RECEIVED,
    PARTIAL_IR_RECEIVED,
    SINGLE_IR_RECEIVED,
    FULL_HR_RECEIVED,
    PARTIAL_HR_RECEIVED,
    SINGLE_HR_RECEIVED,
    RESET_ENERGY,

};

} // namespace pzemCore
#endif