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

#include "blfwk/format_string.h"
#include "blfwk/Logging.h"
#include "blfwk/Updater.h"
#include "bootloader/bootloader.h"
#include "blfwk/DataSource.h"

using namespace blfwk;

//! The supported file types are currently:
//!     - ELF files
//!     - Motorola S-record files
//!     - Binary files
//!     - SB files
//!     - Intel Hex files
//!
//! Any file that is not picked up by the other subclasses will result in a
//! an instance of BinaryDataFile.
//!
//! \return An instance of the correct subclass of SourceFile for the given path.
//!
//! \exception std::runtime_error Thrown if the file cannot be opened.
//!
//! \see blfwk::ELFSourceFile
//! \see blfwk::SRecordSourceFile
//! \see blfwk::BinarySourceFile
//! \see blfwk::SBSourceFile
//! \see blfwk::IntelHexSourceFile
//SourceFile * SourceFile::openFile(const std::string & path)
//{
//	// Search for file using search paths
//	std::string actualPath;
//	bool found = PathSearcher::getGlobalSearcher().search(path, PathSearcher::kFindFile, true, actualPath);
//	if (!found)
//	{
//		throw std::runtime_error(format_string("unable to find file %s\n", path.c_str()));
//	}
//
//	std::ifstream testStream(actualPath.c_str(), std::ios_base::in | std::ios_base::binary);
//	if (!testStream.is_open())
//	{
//		throw std::runtime_error(format_string("failed to open file: %s", actualPath.c_str()));
//	}
//	
//	// catch exceptions so we can close the file stream
//	try
//	{
//		if (ELFSourceFile::isELFFile(testStream))
//		{
//			testStream.close();
//			return new ELFSourceFile(actualPath);
//		}
//		else if (SRecordSourceFile::isSRecordFile(testStream))
//		{
//			testStream.close();
//			return new SRecordSourceFile(actualPath);
//		}
//		else if (SBSourceFile::isSBFile(testStream))
//		{
//			testStream.close();
//			return new SBSourceFile(actualPath);
//		}
////		else if (IntelHexSourceFile::isIntelHexFile(testStream))
////		{
////			testStream.close();
////			return new IntelHexSourceFile(actualPath);
////		}
//		
//		// treat it as a binary file
//		testStream.close();
//		return new BinarySourceFile(actualPath);
//	}
//	catch (...)
//	{
//		testStream.close();
//		throw;
//	}
//}

Updater::Updater(Bootloader * bootloader, const char * filename, uint32_t base_address)
:    m_bootloader(bootloader),
     m_filename(filename),
     m_base_address(base_address),
     m_sector_size(0),
     m_sourceFile(NULL),
     m_operation(kUpdaterOperation_Update),
     m_progressCallback(NULL)
{
    // Create the SourceFile
    m_sourceFile = SourceFile::openFile(m_filename);

    // Initialize the Operation structure.
    m_operation.tasks.push_back(updater_task_t(kUpdaterTask_Erasing, m_sourceFile->getSize()));
    m_operation.tasks.push_back(updater_task_t(kUpdaterTask_Flashing, m_sourceFile->getSize()));
    m_operation.current_task = 0;

    //
    // Get the sector size of the device so we can compute the 
    // address of the Bootloader COnfiguration Block and align
    // erase regions.
    //
    // Inject the get-property(flash-sector-size) command.
    GetProperty cmdGetSectorSize(kProperty_FlashSectorSize);
    Log::info("inject command '%s'\n", cmdGetSectorSize.getName().c_str());
    m_bootloader->inject(cmdGetSectorSize);

    uint32_t fw_status = cmdGetSectorSize.getResponseValues()->at(0);
    if ( fw_status != kStatus_Success )
    {
        throw std::runtime_error(cmdGetSectorSize.getStatusMessage(fw_status));
    }

    m_sector_size = cmdGetSectorSize.getResponseValues()->at(1);
}

Updater::~Updater()
{
    m_bootloader->flush();
}

// See Updater.h for documentation of this method.
void Updater::flashFirmware()
{
    if ( m_sourceFile->getFileType() == SourceFile::source_file_t::kSBSourceFile &&
        isCommandSupported(kCommand_ReceiveSbFile) )
    {
        flashFromSBFile();
    }
    else  if ( m_sourceFile->getFileType() == SourceFile::source_file_t::kELFSourceFile)
    {
        throw std::runtime_error("ELF files are not yet supported.");
    }
    else
    {
        flashFromSourceFile();
    }
}

// See Updater.h for documentation of this method.
bool Updater::isCommandSupported(const cmd_t & command) const
{
    // Inject the get-property(available-commands) command.
    GetProperty cmd(kProperty_AvailableCommands);
    Log::info("inject command '%s'\n", cmd.getName().c_str());
    m_bootloader->inject(cmd);

    uint32_t fw_status = cmd.getResponseValues()->at(0);
    uint32_t fw_response = cmd.getResponseValues()->at(1);
    std::string fw_msg = cmd.getStatusMessage(fw_status);

    // Check the command status
    if ( fw_status != kStatus_Success )
    {
        throw std::runtime_error(fw_msg);
    }

    // See if the command is supported.
    return ((fw_response & command.mask) == command.mask);
}

// See Updater.h for documentation of this method.
void Updater::eraseFlashRegion(uint32_t start, uint32_t length) const
{
    // Align the length to a sector boundary.
    uint32_t alignedLength = (-(-(length) & -(m_sector_size)));

    // Inject the flash-erase-region(start, length) command.
    FlashEraseRegion cmd(start, alignedLength);
    Log::info("inject command '%s'\n", cmd.getName().c_str());
    m_bootloader->inject(cmd);

    uint32_t fw_status = cmd.getResponseValues()->at(0);
    std::string fw_msg = cmd.getStatusMessage(fw_status);

    // Check the command status
    if ( fw_status != kStatus_Success )
    {
        throw std::runtime_error(fw_msg);
    }
}

// See Updater.h for documentation of this method.
void Updater::writeMemory(DataSource::Segment * segment) const
{
    // Inject the write-memory(segment) command.
    WriteMemory cmd(segment);
    Log::info("inject command '%s'\n", cmd.getName().c_str());
    m_bootloader->inject(cmd);

    uint32_t fw_status = cmd.getResponseValues()->at(0);
    std::string fw_msg = cmd.getStatusMessage(fw_status);

    // Check the command status
    if ( fw_status != kStatus_Success )
    {
        throw std::runtime_error(fw_msg);
    }
}

// See Updater.h for documentation of this method.
void Updater::writeMemory(uint32_t address, const uchar_vector_t & data) const
{
    // Inject the write-memory(segment) command.
    WriteMemory cmd(address, data);
    Log::info("inject command '%s'\n", cmd.getName().c_str());
    m_bootloader->inject(cmd);

    uint32_t fw_status = cmd.getResponseValues()->at(0);
    std::string fw_msg = cmd.getStatusMessage(fw_status);

    // Check the command status
    if ( fw_status != kStatus_Success )
    {
        throw std::runtime_error(fw_msg);
    }
}

// See Updater.h for documentation of this method.
void Updater::execute(uint32_t entry_point, uint32_t param, uint32_t stack_pointer) const
{
    // Inject the execute(start_address, param, stack_pointer) command.
    Execute cmd(entry_point, param, stack_pointer);
    Log::info("inject command '%s'\n", cmd.getName().c_str());
    m_bootloader->inject(cmd);

    uint32_t fw_status = cmd.getResponseValues()->at(0);
    std::string fw_msg = cmd.getStatusMessage(fw_status);

    // Check the command status
    if ( fw_status != kStatus_Success )
    {
        throw std::runtime_error(fw_msg);
    }
}

// See Updater.h for documentation of this method.
void Updater::reset() const
{
    // Inject the reset command.
    Reset cmd;
    Log::info("inject command '%s'\n", cmd.getName().c_str());
    m_bootloader->inject(cmd);

    uint32_t fw_status = cmd.getResponseValues()->at(0);
    std::string fw_msg = cmd.getStatusMessage(fw_status);

    // Check the command status
    if ( fw_status != kStatus_Success )
    {
        throw std::runtime_error(fw_msg);
    }
}

// See Updater.h for documentation of this method.
void Updater::flashFromSourceFile()
{
    m_sourceFile->open();

    DataSource * dataSource = m_sourceFile->createDataSource();

    for (uint32_t index = 0; index < dataSource->getSegmentCount(); ++index)
    {
        DataSource::Segment * segment = dataSource->getSegmentAt(index);

        if ( segment->hasNaturalLocation() )
        {
            dataSource->setTarget(new NaturalDataTarget());
        }
        else
        {
            dataSource->setTarget(new ConstantDataTarget(m_base_address));
        }

        // Erase the necessary flash.
        m_operation.current_task = 0;
        m_operation.tasks[m_operation.current_task].current += segment->getLength();
        if ( m_progressCallback )
        {
            m_progressCallback(&m_operation);
        }

        eraseFlashRegion(segment->getBaseAddress(), segment->getLength());
    
        // Write the file to the base address.
        m_operation.current_task = 1;
        m_operation.tasks[m_operation.current_task].current += segment->getLength();
        if ( m_progressCallback )
        {
            m_progressCallback(&m_operation);
        }

        writeMemory(segment);
    }

    m_sourceFile->close();
}

// See Updater.h for documentation of this method.
void Updater::flashFromSBFile()
{
    // Inject the receive-sb-file command.
    ReceiveSbFile cmd(m_filename.c_str());
    Log::info("inject command '%s'\n", cmd.getName().c_str());
    m_bootloader->inject(cmd);

    // print command response values using the Logger.
    cmd.logResponses();

    uint32_t fw_status = cmd.getResponseValues()->at(0);
    std::string fw_msg = cmd.getStatusMessage(fw_status);

    // Check the command status
    if ( fw_status != kStatus_Success )
    {
        throw std::runtime_error(fw_msg);
    }
}
