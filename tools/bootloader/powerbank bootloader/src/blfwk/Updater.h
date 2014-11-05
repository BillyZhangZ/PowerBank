/*
 * Copyright (c) 2013-14, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#if !defined(_Updater_h_)
#define _Updater_h_

#include "host_bootloader.h"
#include "SourceFile.h"
#include "ELFSourceFile.h"
#include "SBSourceFile.h"
#include "SRecordSourceFile.h"

using namespace blfwk;

namespace blfwk
{

//! @name Updater states with descriptions.
//@{
struct updater_enum_t
{ 
    uint32_t value; 
    std::string description;

    updater_enum_t() : value(0), description("") {}

    updater_enum_t(uint32_t value, const char * description)
        : value(value), description(description) {}

    updater_enum_t(const updater_enum_t & old_enum)
    {
        value = old_enum.value;
        description = old_enum.description.c_str();
    }
};
const updater_enum_t kUpdaterState_NotReady  (0,  "Not ready");
const updater_enum_t kUpdaterState_Ready     (1,  "Ready");
const updater_enum_t kUpdaterState_Working   (2,  "Working");
const updater_enum_t kUpdaterState_Idle      (3,  "Idle");
const updater_enum_t kUpdaterState_Complete  (4,  "Complete");
//@}

//! @name Updater tasks with descriptions.
//@{
const updater_enum_t kUpdaterTask_Erasing    (0,  "Erasing");
const updater_enum_t kUpdaterTask_Flashing   (1,  "Flashing");
const updater_enum_t kUpdaterTask_Reseting   (2,  "Reseting");
const updater_enum_t kUpdaterTask_Executing  (3,  "Jumping");

struct updater_task_t
{ 
    updater_enum_t task_desc;
    uint32_t current;
    uint32_t total;

    updater_task_t(const updater_enum_t & task_desc, uint32_t total)
        : task_desc(task_desc), current(0), total(total) {}

    updater_task_t()
        : task_desc(), current(0), total(0) {}

};

typedef std::vector<updater_task_t> updater_task_vector_t;

//! @name Updater operations with descriptions.
//@{
const updater_enum_t kUpdaterOperation_Update (0,  "Update");

//! @name Updater operation.
//@{
struct updater_operation_t
{ 
    updater_enum_t operation_desc;
    updater_task_vector_t tasks;
    uint32_t current_task;
    bool user_stopped;

    uint32_t current()
    {
        uint32_t current = 0;

        for (uint32_t i=0; i < current_task; ++i)
        {
                current += tasks[i].total;
        }
        
        current += tasks[current_task].current;

        return current;
    }

    uint32_t total()
    {
        uint32_t total = 0;
        for (uint32_t i=0; i < tasks.size(); ++i)
        {
            total += tasks[i].total;
        }
        return total;
    }

    updater_operation_t(updater_enum_t operation_desc)
        : operation_desc(operation_desc), tasks(), current_task(0), user_stopped(false) {}

    updater_operation_t(uint32_t value, const char * description)
        : operation_desc(value, description), tasks(), current_task(0), user_stopped(false) {}
};
//@}

/*!
 * \brief Update class contains the functionality necessary to update the
 * firmware on a device running Bootloader.
 *
 * The purpose of this class is to provide a common interface for
 * updating any device running the Kinetis Bootloader from several different file formats.
 */
class Updater
{
public:
    //! \brief Default constructor.
    Updater(Bootloader * bootloader, const char * path, uint32_t base_address);
    
    //! \brief Destructor.
    virtual ~Updater();
    
    //! \name Update API.
    //@{
    //! @brief Type for the progress callback routine.
    typedef bool (*progress_callback_t)(updater_operation_t * op);

    //! @brief Struct used to monitor the Operation progress.
    updater_operation_t m_operation;

    //! \brief  Set the user-defined function to call on progress events.
    //!
    //! \param  callback    The function to callback with the progress data.
    void setCallback(progress_callback_t callback) {  m_progressCallback = callback; }

    //! \brief  Program flash on the device.
    //!
    //! \exception  std::runtime_error  Raised if the file could not be opened successfully.
    //!
    //! \param  callback    The function to callback with the progress data.
    void flashFirmware();

    //! \brief Execute the reset command.
    void reset() const;

    //! \brief Execute the execute command.
    void execute(uint32_t entry_point, uint32_t param=0, uint32_t stack_pointer=0) const;
    //@}
    // 

protected:
    //! \name Bootloader commands
    //@{
    //! \brief Checks if Kinetis Bootloader device supports a given command.
    //!
    //! \exception  std::runtime_error  Thrown if an error occurred while sending the
    //!                                 GetProperty(kProperty_AvailableCommands) bootloader command.
    //!
    //! \param  command The command to check.
    //!
    //! \return true if command is supported, false if not.
    bool isCommandSupported(const cmd_t & command) const;

    //! \brief Execute the FlashEraseRegion bootloader command.
    //!
    //! \exception  std::runtime_error  Thrown if an error occurred while sending the
    //!                                 FlashEraseRegion(start, length) bootloader command.
    //!
    //! \param  start   The beginning address of the memory region to be erased.
    //! \param  length  The length in bytes of the memory region to be erased.
    void eraseFlashRegion(uint32_t start, uint32_t length) const;

    //! \brief Execute the write-memory bootloader command.
    //!
    //! \exception  std::runtime_error  Thrown if an error occurred while sending the
    //!                                 WriteMemory(segment) bootloader command.
    //!
    //! \param [in,out] segment The DatSource::Segment that represents the data to be written to the device.
    void writeMemory(DataSource::Segment * segment) const;

    //! \brief Execute the write-memory bootloader command.
    //!
    //! \exception  std::runtime_error  Thrown if an error occurred while sending the
    //!                                 WriteMemory(vector<uint8_t) bootloader command.
    //!
    //! \param [in] data A vector<uchar> refernce that contains the data to be written to the device.
    //! \param [in] address The address on the device where the data will be written.
    void writeMemory(uint32_t address, const uchar_vector_t & data) const;

    //! \brief Program flash procedure for SourceFile types.
    void flashFromSourceFile();

    //! \brief Program flash procedure for SB files.
    void flashFromSBFile();

protected:
    Bootloader * m_bootloader;              //!< Bootloader object.
    std::string m_filename;                 //!< Path to the file.
    uint32_t m_base_address;                //!< Base address of the image.
    uint32_t m_sector_size;                 //!< Sector size of the flash on the device.
    SourceFile * m_sourceFile;              //!< SourceFile object.
    progress_callback_t m_progressCallback; //!< Callback used to report update progress.
//    smart_ptr<OptionContext> m_options;   //!< Table of option values.
};

}; // namespace blfwk

#endif // _Updater_h_
