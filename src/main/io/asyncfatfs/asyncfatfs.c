/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * This is a FAT16/FAT32 filesystem for SD cards which uses asynchronous operations: The caller need never wait
 * for the SD card to be ready.
 *
 * On top of the regular FAT32 concepts, we add the idea of a "super cluster". Given one FAT sector, a super cluster is
 * the series of clusters which corresponds to all of the cluster entries in that FAT sector. If files are allocated
 * on super-cluster boundaries, they will have FAT sectors which are dedicated to them and independent of all other
 * files.
 *
 * We can pre-allocate a "freefile" which is a file on disk made up of contiguous superclusters. Then when we want
 * to allocate a file on disk, we can carve it out of the freefile, and know that the clusters will be contiguous
 * without needing to read the FAT at all (the freefile's FAT is completely determined from its start cluster and file
 * size, which we get from the directory entry). This allows for extremely fast append-only logging.
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#ifdef AFATFS_DEBUG
#include <signal.h>
#include <stdio.h>
#endif

#include "common/maths.h"
#include "common/time.h"
#include "common/utils.h"

#include "drivers/sdcard.h"

#include "fat_standard.h"

#include "asyncfatfs.h"

#ifdef AFATFS_DEBUG
    #define ONLY_EXPOSE_FOR_TESTING
#else
    #define ONLY_EXPOSE_FOR_TESTING static
#endif

#define AFATFS_NUM_CACHE_SECTORS 11

// FAT filesystems are allowed to differ from these parameters, but we choose not to support those weird filesystems:
#define AFATFS_SECTOR_SIZE  512
#define AFATFS_NUM_FATS     2

#define AFATFS_MAX_OPEN_FILES 3

#define AFATFS_DEFAULT_FILE_DATE FAT_MAKE_DATE(2015, 12, 01)
#define AFATFS_DEFAULT_FILE_TIME FAT_MAKE_TIME(00, 00, 00)

/*
 * How many blocks will we write in a row before we bother using the SDcard's multiple block write method?
 * If this define is omitted, this disables multi-block write.
 */
#define AFATFS_MIN_MULTIPLE_BLOCK_WRITE_COUNT 4

#define AFATFS_FILES_PER_DIRECTORY_SECTOR (AFATFS_SECTOR_SIZE / sizeof(fatDirectoryEntry_t))

#define AFATFS_FAT32_FAT_ENTRIES_PER_SECTOR  (AFATFS_SECTOR_SIZE / sizeof(uint32_t))
#define AFATFS_FAT16_FAT_ENTRIES_PER_SECTOR (AFATFS_SECTOR_SIZE / sizeof(uint16_t))

// We will read from the file
#define AFATFS_FILE_MODE_READ             1
// We will write to the file
#define AFATFS_FILE_MODE_WRITE            2
// We will append to the file, may not be combined with the write flag
#define AFATFS_FILE_MODE_APPEND           4
// File will occupy a series of superclusters (only valid for creating new files):
#define AFATFS_FILE_MODE_CONTIGUOUS       8
// File should be created if it doesn't exist:
#define AFATFS_FILE_MODE_CREATE           16
// The file's directory entry should be locked in cache so we can read it with no latency:
#define AFATFS_FILE_MODE_RETAIN_DIRECTORY 32

// Open the cache sector for read access (it will be read from disk)
#define AFATFS_CACHE_READ         1
// Open the cache sector for write access (it will be marked dirty)
#define AFATFS_CACHE_WRITE        2
// Lock this sector to prevent its state from transitioning (prevent flushes to disk)
#define AFATFS_CACHE_LOCK         4
// Discard this sector in preference to other sectors when it is in the in-sync state
#define AFATFS_CACHE_DISCARDABLE  8
// Increase the retain counter of the cache sector to prevent it from being discarded when in the in-sync state
#define AFATFS_CACHE_RETAIN       16

// Turn the largest free block on the disk into one contiguous file for efficient fragment-free allocation
#define AFATFS_USE_FREEFILE

// When allocating a freefile, leave this many clusters un-allocated for regular files to use
#define AFATFS_FREEFILE_LEAVE_CLUSTERS 100

// Filename in 8.3 format:
#define AFATFS_FREESPACE_FILENAME "FREESPAC.E"

#define AFATFS_INTROSPEC_LOG_FILENAME "ASYNCFAT.LOG"

typedef enum {
    AFATFS_SAVE_DIRECTORY_NORMAL,
    AFATFS_SAVE_DIRECTORY_FOR_CLOSE,
    AFATFS_SAVE_DIRECTORY_DELETED
} afatfsSaveDirectoryEntryMode_e;

typedef enum {
    AFATFS_CACHE_STATE_EMPTY,
    AFATFS_CACHE_STATE_IN_SYNC,
    AFATFS_CACHE_STATE_READING,
    AFATFS_CACHE_STATE_WRITING,
    AFATFS_CACHE_STATE_DIRTY
} afatfsCacheBlockState_e;

typedef enum {
    AFATFS_FILE_TYPE_NONE,
    AFATFS_FILE_TYPE_NORMAL,
    AFATFS_FILE_TYPE_FAT16_ROOT_DIRECTORY,
    AFATFS_FILE_TYPE_DIRECTORY
} afatfsFileType_e;

typedef enum {
    CLUSTER_SEARCH_FREE_AT_BEGINNING_OF_FAT_SECTOR,
    CLUSTER_SEARCH_FREE,
    CLUSTER_SEARCH_OCCUPIED
} afatfsClusterSearchCondition_e;

enum {
    AFATFS_CREATEFILE_PHASE_INITIAL = 0,
    AFATFS_CREATEFILE_PHASE_FIND_FILE,
    AFATFS_CREATEFILE_PHASE_CREATE_NEW_FILE,
    AFATFS_CREATEFILE_PHASE_SUCCESS,
    AFATFS_CREATEFILE_PHASE_FAILURE
};

typedef enum {
    AFATFS_FIND_CLUSTER_IN_PROGRESS,
    AFATFS_FIND_CLUSTER_FOUND,
    AFATFS_FIND_CLUSTER_FATAL,
    AFATFS_FIND_CLUSTER_NOT_FOUND
} afatfsFindClusterStatus_e;

struct afatfsFileOperation_t;

typedef union afatfsFATSector_t {
    uint8_t *bytes;
    uint16_t *fat16;
    uint32_t *fat32;
} afatfsFATSector_t;

typedef struct afatfsCacheBlockDescriptor_t {
    /*
     * The physical sector index on disk that this cached block corresponds to
     */
    uint32_t sectorIndex;

    // We use an increasing timestamp to identify cache access times.

    // This is the timestamp that this sector was first marked dirty at (so we can flush sectors in write-order).
    uint32_t writeTimestamp;

    // This is the last time the sector was accessed
    uint32_t accessTimestamp;

    /* This is set to non-zero when we expect to write a consecutive series of this many blocks (including this block),
     * so we will tell the SD-card to pre-erase those blocks.
     *
     * This counter only needs to be set on the first block of a consecutive write (though setting it, appropriately
     * decreased, on the subsequent blocks won't hurt).
     */
    uint16_t consecutiveEraseBlockCount;

    afatfsCacheBlockState_e state;

    /*
     * The state of this block must not transition (do not flush to disk, do not discard). This is useful for a sector
     * which is currently being written to by the application (so flushing it would be a waste of time).
     *
     * This is a binary state rather than a counter because we assume that only one party will be responsible for and
     * so consider locking a given sector.
     */
    unsigned locked:1;

    /*
     * A counter for how many parties want this sector to be retained in memory (not discarded). If this value is
     * non-zero, the sector may be flushed to disk if dirty but must remain in the cache. This is useful if we require
     * a directory sector to be cached in order to meet our response time requirements.
     */
    unsigned retainCount:6;

    /*
     * If this block is in the In Sync state, it should be discarded from the cache in preference to other blocks.
     * This is useful for data that we don't expect to read again, e.g. data written to an append-only file. This hint
     * is overridden by the locked and retainCount flags.
     */
    unsigned discardable:1;
} afatfsCacheBlockDescriptor_t;

typedef enum {
    AFATFS_FAT_PATTERN_UNTERMINATED_CHAIN,
    AFATFS_FAT_PATTERN_TERMINATED_CHAIN,
    AFATFS_FAT_PATTERN_FREE
} afatfsFATPattern_e;

typedef enum {
    AFATFS_FREE_SPACE_SEARCH_PHASE_FIND_HOLE,
    AFATFS_FREE_SPACE_SEARCH_PHASE_GROW_HOLE
} afatfsFreeSpaceSearchPhase_e;

typedef struct afatfsFreeSpaceSearch_t {
    uint32_t candidateStart;
    uint32_t candidateEnd;
    uint32_t bestGapStart;
    uint32_t bestGapLength;
    afatfsFreeSpaceSearchPhase_e phase;
} afatfsFreeSpaceSearch_t;

typedef struct afatfsFreeSpaceFAT_t {
    uint32_t startCluster;
    uint32_t endCluster;
} afatfsFreeSpaceFAT_t;

typedef struct afatfsCreateFile_t {
    afatfsFileCallback_t callback;

    uint8_t phase;
    uint8_t filename[FAT_FILENAME_LENGTH];
} afatfsCreateFile_t;

typedef struct afatfsSeek_t {
    afatfsFileCallback_t callback;

    uint32_t seekOffset;
} afatfsSeek_t;

typedef enum {
    AFATFS_APPEND_SUPERCLUSTER_PHASE_INIT = 0,
    AFATFS_APPEND_SUPERCLUSTER_PHASE_UPDATE_FREEFILE_DIRECTORY,
    AFATFS_APPEND_SUPERCLUSTER_PHASE_UPDATE_FAT,
    AFATFS_APPEND_SUPERCLUSTER_PHASE_UPDATE_FILE_DIRECTORY
} afatfsAppendSuperclusterPhase_e;

typedef struct afatfsAppendSupercluster_t {
    uint32_t previousCluster;
    uint32_t fatRewriteStartCluster;
    uint32_t fatRewriteEndCluster;
    afatfsAppendSuperclusterPhase_e phase;
} afatfsAppendSupercluster_t;

typedef enum {
    AFATFS_APPEND_FREE_CLUSTER_PHASE_INITIAL = 0,
    AFATFS_APPEND_FREE_CLUSTER_PHASE_FIND_FREESPACE = 0,
    AFATFS_APPEND_FREE_CLUSTER_PHASE_UPDATE_FAT1,
    AFATFS_APPEND_FREE_CLUSTER_PHASE_UPDATE_FAT2,
    AFATFS_APPEND_FREE_CLUSTER_PHASE_UPDATE_FILE_DIRECTORY,
    AFATFS_APPEND_FREE_CLUSTER_PHASE_COMPLETE,
    AFATFS_APPEND_FREE_CLUSTER_PHASE_FAILURE
} afatfsAppendFreeClusterPhase_e;

typedef struct afatfsAppendFreeCluster_t {
    uint32_t previousCluster;
    uint32_t searchCluster;
    afatfsAppendFreeClusterPhase_e phase;
} afatfsAppendFreeCluster_t;

typedef enum {
    AFATFS_EXTEND_SUBDIRECTORY_PHASE_INITIAL = 0,
    AFATFS_EXTEND_SUBDIRECTORY_PHASE_ADD_FREE_CLUSTER = 0,
    AFATFS_EXTEND_SUBDIRECTORY_PHASE_WRITE_SECTORS,
    AFATFS_EXTEND_SUBDIRECTORY_PHASE_SUCCESS,
    AFATFS_EXTEND_SUBDIRECTORY_PHASE_FAILURE
} afatfsExtendSubdirectoryPhase_e;

typedef struct afatfsExtendSubdirectory_t {
    // We need to call this as a sub-operation so we have it as our first member to be compatible with its memory layout:
    afatfsAppendFreeCluster_t appendFreeCluster;

    afatfsExtendSubdirectoryPhase_e phase;

    uint32_t parentDirectoryCluster;
    afatfsFileCallback_t callback;
} afatfsExtendSubdirectory_t;

typedef enum {
    AFATFS_TRUNCATE_FILE_INITIAL = 0,
    AFATFS_TRUNCATE_FILE_UPDATE_DIRECTORY = 0,
    AFATFS_TRUNCATE_FILE_ERASE_FAT_CHAIN_NORMAL,
#ifdef AFATFS_USE_FREEFILE
    AFATFS_TRUNCATE_FILE_ERASE_FAT_CHAIN_CONTIGUOUS,
    AFATFS_TRUNCATE_FILE_PREPEND_TO_FREEFILE,
#endif
    AFATFS_TRUNCATE_FILE_SUCCESS
} afatfsTruncateFilePhase_e;

typedef struct afatfsTruncateFile_t {
    uint32_t startCluster; // First cluster to erase
    uint32_t currentCluster; // Used to mark progress
    uint32_t endCluster; // Optional, for contiguous files set to 1 past the end cluster of the file, otherwise set to 0
    afatfsFileCallback_t callback;
    afatfsTruncateFilePhase_e phase;
} afatfsTruncateFile_t;

typedef enum {
    AFATFS_DELETE_FILE_DELETE_DIRECTORY_ENTRY,
    AFATFS_DELETE_FILE_DEALLOCATE_CLUSTERS
} afatfsDeleteFilePhase_e;

typedef struct afatfsDeleteFile_t {
    afatfsTruncateFile_t truncateFile;
    afatfsCallback_t callback;
} afatfsUnlinkFile_t;

typedef struct afatfsCloseFile_t {
    afatfsCallback_t callback;
} afatfsCloseFile_t;

typedef enum {
    AFATFS_FILE_OPERATION_NONE,
    AFATFS_FILE_OPERATION_CREATE_FILE,
    AFATFS_FILE_OPERATION_SEEK, // Seek the file's cursorCluster forwards by seekOffset bytes
    AFATFS_FILE_OPERATION_CLOSE,
    AFATFS_FILE_OPERATION_TRUNCATE,
    AFATFS_FILE_OPERATION_UNLINK,
#ifdef AFATFS_USE_FREEFILE
    AFATFS_FILE_OPERATION_APPEND_SUPERCLUSTER,
    AFATFS_FILE_OPERATION_LOCKED,
#endif
    AFATFS_FILE_OPERATION_APPEND_FREE_CLUSTER,
    AFATFS_FILE_OPERATION_EXTEND_SUBDIRECTORY
} afatfsFileOperation_e;

typedef struct afatfsFileOperation_t {
    afatfsFileOperation_e operation;
    union {
        afatfsCreateFile_t createFile;
        afatfsSeek_t seek;
        afatfsAppendSupercluster_t appendSupercluster;
        afatfsAppendFreeCluster_t appendFreeCluster;
        afatfsExtendSubdirectory_t extendSubdirectory;
        afatfsUnlinkFile_t unlinkFile;
        afatfsTruncateFile_t truncateFile;
        afatfsCloseFile_t closeFile;
    } state;
} afatfsFileOperation_t;

typedef struct afatfsFile_t {
    afatfsFileType_e type;

    // The byte offset of the cursor within the file
    uint32_t cursorOffset;

    /* The file size in bytes as seen by users of the filesystem (the exact length of the file they've written).
     *
     * This is only used by users of the filesystem, not us, so it only needs to be up to date for fseek() (to clip
     * seeks to the EOF), fread(), feof(), and fclose() (which writes the logicalSize to the directory).
     *
     * It becomes out of date when we fwrite() to extend the length of the file. In this situation, feof() is properly
     * true, so we don't have to update the logicalSize for fread() or feof() to get the correct result. We only need
     * to update it when we seek backwards (so we don't forget the logical EOF position), or fclose().
     */
    uint32_t logicalSize;

    /* The allocated size in bytes based on how many clusters have been assigned to the file. Always a multiple of
     * the cluster size.
     *
     * This is an underestimate for existing files, because we don't bother to check precisely how long the chain is
     * at the time the file is opened (it might be longer than needed to contain the logical size), but assuming the
     * filesystem metadata is correct, it should always be at least as many clusters as needed to contain logicalSize.
     *
     * Since this is an estimate, we only use it to exaggerate the filesize in the directory entry of a file that is
     * currently being written (so that the final cluster of the file will be entirely readable if power is lost before
     * we can could update the directory entry with a new logicalSize).
     */
    uint32_t physicalSize;

    /*
     * The cluster that the file pointer is currently within. When seeking to the end of the file, this will be
     * set to zero.
     */
    uint32_t cursorCluster;

    /*
     * The cluster before the one the file pointer is inside. This is set to zero when at the start of the file.
     */
    uint32_t cursorPreviousCluster;

    uint8_t mode; // A combination of AFATFS_FILE_MODE_* flags
    uint8_t attrib; // Combination of FAT_FILE_ATTRIBUTE_* flags for the directory entry of this file

    /* We hold on to one sector entry in the cache and remember its index here. The cache is invalidated when we
     * seek across a sector boundary. This allows fwrite() to complete faster because it doesn't need to check the
     * cache on every call.
     */
    int8_t writeLockedCacheIndex;
    // Ditto for fread():
    int8_t readRetainCacheIndex;

    // The position of our directory entry on the disk (so we can update it without consulting a parent directory file)
    afatfsDirEntryPointer_t directoryEntryPos;

    // The first cluster number of the file, or 0 if this file is empty
    uint32_t firstCluster;

    // State for a queued operation on the file
    struct afatfsFileOperation_t operation;
} afatfsFile_t;

typedef enum {
    AFATFS_INITIALIZATION_READ_MBR,
    AFATFS_INITIALIZATION_READ_VOLUME_ID,

#ifdef AFATFS_USE_FREEFILE
    AFATFS_INITIALIZATION_FREEFILE_CREATE,
    AFATFS_INITIALIZATION_FREEFILE_CREATING,
    AFATFS_INITIALIZATION_FREEFILE_FAT_SEARCH,
    AFATFS_INITIALIZATION_FREEFILE_UPDATE_FAT,
    AFATFS_INITIALIZATION_FREEFILE_SAVE_DIR_ENTRY,
    AFATFS_INITIALIZATION_FREEFILE_LAST = AFATFS_INITIALIZATION_FREEFILE_SAVE_DIR_ENTRY,
#endif

#ifdef AFATFS_USE_INTROSPECTIVE_LOGGING
    AFATFS_INITIALIZATION_INTROSPEC_LOG_CREATE,
    AFATFS_INITIALIZATION_INTROSPEC_LOG_CREATING,
#endif

    AFATFS_INITIALIZATION_DONE
} afatfsInitializationPhase_e;

typedef struct afatfs_t {
    fatFilesystemType_e filesystemType;

    afatfsFilesystemState_e filesystemState;
    afatfsInitializationPhase_e initPhase;

    // State used during FS initialisation where only one member of the union is used at a time
#ifdef AFATFS_USE_FREEFILE
    union {
        afatfsFreeSpaceSearch_t freeSpaceSearch;
        afatfsFreeSpaceFAT_t freeSpaceFAT;
    } initState;
#endif

#ifdef STM32H7
    uint8_t *cache;
#else
    uint8_t cache[AFATFS_SECTOR_SIZE * AFATFS_NUM_CACHE_SECTORS];
#endif
    afatfsCacheBlockDescriptor_t cacheDescriptor[AFATFS_NUM_CACHE_SECTORS];
    uint32_t cacheTimer;

    int cacheDirtyEntries; // The number of cache entries in the AFATFS_CACHE_STATE_DIRTY state
    bool cacheFlushInProgress;

    afatfsFile_t openFiles[AFATFS_MAX_OPEN_FILES];

#ifdef AFATFS_USE_FREEFILE
    afatfsFile_t freeFile;
#endif

#ifdef AFATFS_USE_INTROSPECTIVE_LOGGING
    afatfsFile_t introSpecLog;
#endif

    afatfsError_e lastError;

    bool filesystemFull;

    // The current working directory:
    afatfsFile_t currentDirectory;

    uint32_t partitionStartSector; // The physical sector that the first partition on the device begins at

    uint32_t fatStartSector; // The first sector of the first FAT
    uint32_t fatSectors;     // The size in sectors of a single FAT

    /*
     * Number of clusters available for storing user data. Note that clusters are numbered starting from 2, so the
     * index of the last cluster on the volume is numClusters + 1 !!!
     */
    uint32_t numClusters;
    uint32_t clusterStartSector; // The physical sector that the clusters area begins at
    uint32_t sectorsPerCluster;

    /*
     * Number of the cluster we last allocated (i.e. free->occupied). Searches for a free cluster will begin after this
     * cluster.
     */
    uint32_t lastClusterAllocated;

    /* Mask to be ANDed with a byte offset within a file to give the offset within the cluster */
    uint32_t byteInClusterMask;

    uint32_t rootDirectoryCluster; // Present on FAT32 and set to zero for FAT16
    uint32_t rootDirectorySectors; // Zero on FAT32, for FAT16 the number of sectors that the root directory occupies
} afatfs_t;

#ifdef STM32H7
static DMA_DATA_ZERO_INIT uint8_t afatfs_cache[AFATFS_SECTOR_SIZE * AFATFS_NUM_CACHE_SECTORS] __attribute__((aligned(32)));
#endif

static afatfs_t afatfs;

static void afatfs_fileOperationContinue(afatfsFile_t *file);
static uint8_t* afatfs_fileLockCursorSectorForWrite(afatfsFilePtr_t file);
static uint8_t* afatfs_fileRetainCursorSectorForRead(afatfsFilePtr_t file);

static uint32_t roundUpTo(uint32_t value, uint32_t rounding)
{
    uint32_t remainder = value % rounding;

    if (remainder > 0) {
        value += rounding - remainder;
    }

    return value;
}

static bool isPowerOfTwo(unsigned int x)
{
    return ((x != 0) && ((x & (~x + 1)) == x));
}

/**
 * Check for conditions that should always be true (and if otherwise mean a bug or a corrupt filesystem).
 *
 * If the condition is false, the filesystem is marked as being in a fatal state.
 *
 * Returns the value of the condition.
 */
static bool afatfs_assert(bool condition)
{
    if (!condition) {
        if (afatfs.lastError == AFATFS_ERROR_NONE) {
            afatfs.lastError = AFATFS_ERROR_GENERIC;
        }
        afatfs.filesystemState = AFATFS_FILESYSTEM_STATE_FATAL;
#ifdef AFATFS_DEBUG
        raise(SIGTRAP);
#endif
    }

    return condition;
}

static bool afatfs_fileIsBusy(afatfsFilePtr_t file)
{
    return file->operation.operation != AFATFS_FILE_OPERATION_NONE;
}

/**
 * The number of FAT table entries that fit within one AFATFS sector size.
 *
 * Note that this is the same as the number of clusters in an AFATFS supercluster.
 */
static uint32_t afatfs_fatEntriesPerSector(void)
{
    return afatfs.filesystemType == FAT_FILESYSTEM_TYPE_FAT32 ? AFATFS_FAT32_FAT_ENTRIES_PER_SECTOR : AFATFS_FAT16_FAT_ENTRIES_PER_SECTOR;
}

/**
 * Size of a FAT cluster in bytes
 */
ONLY_EXPOSE_FOR_TESTING
uint32_t afatfs_clusterSize(void)
{
    return afatfs.sectorsPerCluster * AFATFS_SECTOR_SIZE;
}

/**
 * Given a byte offset within a file, return the byte offset of that position within the cluster it belongs to.
 */
static uint32_t afatfs_byteIndexInCluster(uint32_t byteOffset)
{
    return afatfs.byteInClusterMask & byteOffset;
}

/**
 * Given a byte offset within a file, return the index of the sector within the cluster it belongs to.
 */
static uint32_t afatfs_sectorIndexInCluster(uint32_t byteOffset)
{
    return afatfs_byteIndexInCluster(byteOffset) / AFATFS_SECTOR_SIZE;
}

// Get the buffer memory for the cache entry of the given index.
static uint8_t *afatfs_cacheSectorGetMemory(int cacheEntryIndex)
{
    return afatfs.cache + cacheEntryIndex * AFATFS_SECTOR_SIZE;
}

static int afatfs_getCacheDescriptorIndexForBuffer(uint8_t *memory)
{
    int index = (memory - afatfs.cache) / AFATFS_SECTOR_SIZE;

    if (afatfs_assert(index >= 0 && index < AFATFS_NUM_CACHE_SECTORS)) {
        return index;
    } else {
        return -1;
    }
}

static afatfsCacheBlockDescriptor_t* afatfs_getCacheDescriptorForBuffer(uint8_t *memory)
{
    return afatfs.cacheDescriptor + afatfs_getCacheDescriptorIndexForBuffer(memory);
}

static void afatfs_cacheSectorMarkDirty(afatfsCacheBlockDescriptor_t *descriptor)
{
    if (descriptor->state != AFATFS_CACHE_STATE_DIRTY) {
        descriptor->writeTimestamp = ++afatfs.cacheTimer;
        descriptor->state = AFATFS_CACHE_STATE_DIRTY;
        afatfs.cacheDirtyEntries++;
    }
}

static void afatfs_cacheSectorInit(afatfsCacheBlockDescriptor_t *descriptor, uint32_t sectorIndex, bool locked)
{
    descriptor->sectorIndex = sectorIndex;

    descriptor->accessTimestamp = descriptor->writeTimestamp = ++afatfs.cacheTimer;

    descriptor->consecutiveEraseBlockCount = 0;

    descriptor->state = AFATFS_CACHE_STATE_EMPTY;

    descriptor->locked = locked;
    descriptor->retainCount = 0;
    descriptor->discardable = 0;
}

/**
 * Called by the SD card driver when one of our read operations completes.
 */
static void afatfs_sdcardReadComplete(sdcardBlockOperation_e operation, uint32_t sectorIndex, uint8_t *buffer, uint32_t callbackData)
{
    (void) operation;
    (void) callbackData;

    for (int i = 0; i < AFATFS_NUM_CACHE_SECTORS; i++) {
        if (afatfs.cacheDescriptor[i].state != AFATFS_CACHE_STATE_EMPTY
            && afatfs.cacheDescriptor[i].sectorIndex == sectorIndex
        ) {
            if (buffer == NULL) {
                // Read failed, mark the sector as empty and whoever asked for it will ask for it again later to retry
                afatfs.cacheDescriptor[i].state = AFATFS_CACHE_STATE_EMPTY;
            } else {
                afatfs_assert(afatfs_cacheSectorGetMemory(i) == buffer && afatfs.cacheDescriptor[i].state == AFATFS_CACHE_STATE_READING);

                afatfs.cacheDescriptor[i].state = AFATFS_CACHE_STATE_IN_SYNC;
            }

            break;
        }
    }
}

/**
 * Called by the SD card driver when one of our write operations completes.
 */
static void afatfs_sdcardWriteComplete(sdcardBlockOperation_e operation, uint32_t sectorIndex, uint8_t *buffer, uint32_t callbackData)
{
    (void) operation;
    (void) callbackData;

    afatfs.cacheFlushInProgress = false;

    for (int i = 0; i < AFATFS_NUM_CACHE_SECTORS; i++) {
        /* Keep in mind that someone may have marked the sector as dirty after writing had already begun. In this case we must leave
         * it marked as dirty because those modifications may have been made too late to make it to the disk!
         */
        if (afatfs.cacheDescriptor[i].sectorIndex == sectorIndex
            && afatfs.cacheDescriptor[i].state == AFATFS_CACHE_STATE_WRITING
        ) {
            if (buffer == NULL) {
                // Write failed, remark the sector as dirty
                afatfs.cacheDescriptor[i].state = AFATFS_CACHE_STATE_DIRTY;
                afatfs.cacheDirtyEntries++;
            } else {
                afatfs_assert(afatfs_cacheSectorGetMemory(i) == buffer);

                afatfs.cacheDescriptor[i].state = AFATFS_CACHE_STATE_IN_SYNC;
            }
            break;
        }
    }
}

/**
 * Attempt to flush the dirty cache entry with the given index to the SDcard.
 */
static void afatfs_cacheFlushSector(int cacheIndex)
{
    afatfsCacheBlockDescriptor_t *cacheDescriptor = &afatfs.cacheDescriptor[cacheIndex];

#ifdef AFATFS_MIN_MULTIPLE_BLOCK_WRITE_COUNT
    if (cacheDescriptor->consecutiveEraseBlockCount) {
        sdcard_beginWriteBlocks(cacheDescriptor->sectorIndex, cacheDescriptor->consecutiveEraseBlockCount);
    }
#endif

    switch (sdcard_writeBlock(cacheDescriptor->sectorIndex, afatfs_cacheSectorGetMemory(cacheIndex), afatfs_sdcardWriteComplete, 0)) {
        case SDCARD_OPERATION_IN_PROGRESS:
            // The card will call us back later when the buffer transmission finishes
            afatfs.cacheDirtyEntries--;
            cacheDescriptor->state = AFATFS_CACHE_STATE_WRITING;
            afatfs.cacheFlushInProgress = true;
            break;

        case SDCARD_OPERATION_SUCCESS:
            // Buffer is already transmitted
            afatfs.cacheDirtyEntries--;
            cacheDescriptor->state = AFATFS_CACHE_STATE_IN_SYNC;
            break;

        case SDCARD_OPERATION_BUSY:
        case SDCARD_OPERATION_FAILURE:
        default:
            ;
    }
}

// Check whether every sector in the cache that can be flushed has been synchronized
bool afatfs_sectorCacheInSync(void)
{
    for (int i = 0; i < AFATFS_NUM_CACHE_SECTORS; i++) {
        if ((afatfs.cacheDescriptor[i].state == AFATFS_CACHE_STATE_WRITING) ||
            ((afatfs.cacheDescriptor[i].state == AFATFS_CACHE_STATE_DIRTY) && !afatfs.cacheDescriptor[i].locked)) {
            return false;
        }
    }
    return true;
}

/**
 * Find a sector in the cache which corresponds to the given physical sector index, or NULL if the sector isn't
 * cached. Note that the cached sector could be in any state including completely empty.
 */
static afatfsCacheBlockDescriptor_t* afatfs_findCacheSector(uint32_t sectorIndex)
{
    for (int i = 0; i < AFATFS_NUM_CACHE_SECTORS; i++) {
        if (afatfs.cacheDescriptor[i].sectorIndex == sectorIndex) {
            return &afatfs.cacheDescriptor[i];
        }
    }

    return NULL;
}

/**
 * Find or allocate a cache sector for the given sector index on disk. Returns a block which matches one of these
 * conditions (in descending order of preference):
 *
 * - The requested sector that already exists in the cache
 * - The index of an empty sector
 * - The index of a synced discardable sector
 * - The index of the oldest synced sector
 *
 * Otherwise it returns -1 to signal failure (cache is full!)
 */
static int afatfs_allocateCacheSector(uint32_t sectorIndex)
{
    int allocateIndex;
    int emptyIndex = -1, discardableIndex = -1;

    uint32_t oldestSyncedSectorLastUse = 0xFFFFFFFF;
    int oldestSyncedSectorIndex = -1;

    if (
        !afatfs_assert(
            afatfs.numClusters == 0 // We're unable to check sector bounds during startup since we haven't read volume label yet
            || sectorIndex < afatfs.clusterStartSector + afatfs.numClusters * afatfs.sectorsPerCluster
        )
    ) {
        return -1;
    }

    for (int i = 0; i < AFATFS_NUM_CACHE_SECTORS; i++) {
        if (afatfs.cacheDescriptor[i].sectorIndex == sectorIndex) {
            /*
             * If the sector is actually empty then do a complete re-init of it just like the standard
             * empty case. (Sectors marked as empty should be treated as if they don't have a block index assigned)
             */
            if (afatfs.cacheDescriptor[i].state == AFATFS_CACHE_STATE_EMPTY) {
                emptyIndex = i;
                break;
            }

            // Bump the last access time
            afatfs.cacheDescriptor[i].accessTimestamp = ++afatfs.cacheTimer;
            return i;
        }

        switch (afatfs.cacheDescriptor[i].state) {
            case AFATFS_CACHE_STATE_EMPTY:
                emptyIndex = i;
            break;
            case AFATFS_CACHE_STATE_IN_SYNC:
                // Is this a synced sector that we could evict from the cache?
                if (!afatfs.cacheDescriptor[i].locked && afatfs.cacheDescriptor[i].retainCount == 0) {
                    if (afatfs.cacheDescriptor[i].discardable) {
                        discardableIndex = i;
                    } else if (afatfs.cacheDescriptor[i].accessTimestamp < oldestSyncedSectorLastUse) {
                        // This is older than last block we decided to evict, so evict this one in preference
                        oldestSyncedSectorLastUse = afatfs.cacheDescriptor[i].accessTimestamp;
                        oldestSyncedSectorIndex = i;
                    }
                }
            break;
            default:
                ;
        }
    }

    if (emptyIndex > -1) {
        allocateIndex = emptyIndex;
    } else if (discardableIndex > -1) {
        allocateIndex = discardableIndex;
    } else if (oldestSyncedSectorIndex > -1) {
        allocateIndex = oldestSyncedSectorIndex;
    } else {
        allocateIndex = -1;
    }

    if (allocateIndex > -1) {
        afatfs_cacheSectorInit(&afatfs.cacheDescriptor[allocateIndex], sectorIndex, false);
    }

    return allocateIndex;
}

/**
 * Attempt to flush dirty cache pages out to the sdcard, returning true if all flushable data has been flushed.
 */
bool afatfs_flush(void)
{
    if (afatfs.cacheDirtyEntries > 0) {
        // Flush the oldest flushable sector
        uint32_t earliestSectorTime = 0xFFFFFFFF;
        int earliestSectorIndex = -1;

        for (int i = 0; i < AFATFS_NUM_CACHE_SECTORS; i++) {
            if (afatfs.cacheDescriptor[i].state == AFATFS_CACHE_STATE_DIRTY && !afatfs.cacheDescriptor[i].locked
                && (earliestSectorIndex == -1 || afatfs.cacheDescriptor[i].writeTimestamp < earliestSectorTime)
            ) {
                earliestSectorIndex = i;
                earliestSectorTime = afatfs.cacheDescriptor[i].writeTimestamp;
            }
        }

        if (earliestSectorIndex > -1) {
            afatfs_cacheFlushSector(earliestSectorIndex);

            // That flush will take time to complete so we may as well tell caller to come back later
            return false;
        }
    }

    return true;
}

/**
 * Returns true if either the freefile or the regular cluster pool has been exhausted during a previous write operation.
 */
bool afatfs_isFull(void)
{
    return afatfs.filesystemFull;
}

/**
 * Get the physical sector number that corresponds to the FAT sector of the given fatSectorIndex within the given
 * FAT (fatIndex may be 0 or 1). (0, 0) gives the first sector of the first FAT.
 */
static uint32_t afatfs_fatSectorToPhysical(int fatIndex, uint32_t fatSectorIndex)
{
    return afatfs.fatStartSector + (fatIndex ? afatfs.fatSectors : 0) + fatSectorIndex;
}

static uint32_t afatfs_fileClusterToPhysical(uint32_t clusterNumber, uint32_t sectorIndex)
{
    return afatfs.clusterStartSector + (clusterNumber - 2) * afatfs.sectorsPerCluster + sectorIndex;
}

static uint32_t afatfs_fileGetCursorPhysicalSector(afatfsFilePtr_t file)
{
    if (file->type == AFATFS_FILE_TYPE_FAT16_ROOT_DIRECTORY) {
        return afatfs.fatStartSector + AFATFS_NUM_FATS * afatfs.fatSectors + file->cursorOffset / AFATFS_SECTOR_SIZE;
    } else {
        uint32_t cursorSectorInCluster = afatfs_sectorIndexInCluster(file->cursorOffset);
        return afatfs_fileClusterToPhysical(file->cursorCluster, cursorSectorInCluster);
    }
}

/**
 * Sector here is the sector index within the cluster.
 */
static void afatfs_fileGetCursorClusterAndSector(afatfsFilePtr_t file, uint32_t *cluster, uint16_t *sector)
{
    *cluster = file->cursorCluster;

    if (file->type == AFATFS_FILE_TYPE_FAT16_ROOT_DIRECTORY) {
        *sector = file->cursorOffset / AFATFS_SECTOR_SIZE;
    } else {
        *sector = afatfs_sectorIndexInCluster(file->cursorOffset);
    }
}

/**
 * Get a cache entry for the given sector and store a pointer to the cached memory in *buffer.
 *
 * physicalSectorIndex - The index of the sector in the SD card to cache
 * sectorflags         - A union of AFATFS_CACHE_* constants that says which operations the sector will be cached for.
 * buffer              - A pointer to the 512-byte memory buffer for the sector will be stored here upon success
 * eraseCount          - For write operations, set to a non-zero number to hint that we plan to write that many sectors
 *                       consecutively (including this sector)
 *
 * Returns:
 *     AFATFS_OPERATION_SUCCESS     - On success
 *     AFATFS_OPERATION_IN_PROGRESS - Card is busy, call again later
 *     AFATFS_OPERATION_FAILURE     - When the filesystem encounters a fatal error
 */
static afatfsOperationStatus_e afatfs_cacheSector(uint32_t physicalSectorIndex, uint8_t **buffer, uint8_t sectorFlags, uint32_t eraseCount)
{
    // We never write to the MBR, so any attempt to write there is an asyncfatfs bug
    if (!afatfs_assert((sectorFlags & AFATFS_CACHE_WRITE) == 0 || physicalSectorIndex != 0)) {
        return AFATFS_OPERATION_FAILURE;
    }

    int cacheSectorIndex = afatfs_allocateCacheSector(physicalSectorIndex);

    if (cacheSectorIndex == -1) {
        // We don't have enough free cache to service this request right now, try again later
        return AFATFS_OPERATION_IN_PROGRESS;
    }

    switch (afatfs.cacheDescriptor[cacheSectorIndex].state) {
        case AFATFS_CACHE_STATE_READING:
            return AFATFS_OPERATION_IN_PROGRESS;
        break;

        case AFATFS_CACHE_STATE_EMPTY:
            if ((sectorFlags & AFATFS_CACHE_READ) != 0) {
                if (sdcard_readBlock(physicalSectorIndex, afatfs_cacheSectorGetMemory(cacheSectorIndex), afatfs_sdcardReadComplete, 0)) {
                    afatfs.cacheDescriptor[cacheSectorIndex].state = AFATFS_CACHE_STATE_READING;
                }
                return AFATFS_OPERATION_IN_PROGRESS;
            }

            // We only get to decide these fields if we're the first ones to cache the sector:
            afatfs.cacheDescriptor[cacheSectorIndex].discardable = (sectorFlags & AFATFS_CACHE_DISCARDABLE) != 0 ? 1 : 0;

#ifdef AFATFS_MIN_MULTIPLE_BLOCK_WRITE_COUNT
            // Don't bother pre-erasing for small block sequences
            if (eraseCount < AFATFS_MIN_MULTIPLE_BLOCK_WRITE_COUNT) {
                eraseCount = 0;
            } else {
                eraseCount = MIN(eraseCount, (uint32_t)UINT16_MAX); // If caller asked for a longer chain of sectors we silently truncate that here
            }

            afatfs.cacheDescriptor[cacheSectorIndex].consecutiveEraseBlockCount = eraseCount;
#endif

            FALLTHROUGH;

        case AFATFS_CACHE_STATE_WRITING:
        case AFATFS_CACHE_STATE_IN_SYNC:
            if ((sectorFlags & AFATFS_CACHE_WRITE) != 0) {
                afatfs_cacheSectorMarkDirty(&afatfs.cacheDescriptor[cacheSectorIndex]);
            }
            FALLTHROUGH;

        case AFATFS_CACHE_STATE_DIRTY:
            if ((sectorFlags & AFATFS_CACHE_LOCK) != 0) {
                afatfs.cacheDescriptor[cacheSectorIndex].locked = 1;
            }
            if ((sectorFlags & AFATFS_CACHE_RETAIN) != 0) {
                afatfs.cacheDescriptor[cacheSectorIndex].retainCount++;
            }

            *buffer = afatfs_cacheSectorGetMemory(cacheSectorIndex);

            return AFATFS_OPERATION_SUCCESS;
        break;

        default:
            // Cache block in unknown state, should never happen
            afatfs_assert(false);
            return AFATFS_OPERATION_FAILURE;
    }
}

/**
 * Parse the details out of the given MBR sector (512 bytes long). Return true if a compatible filesystem was found.
 */
static bool afatfs_parseMBR(const uint8_t *sector)
{
    // Check MBR signature
    if (sector[AFATFS_SECTOR_SIZE - 2] != 0x55 || sector[AFATFS_SECTOR_SIZE - 1] != 0xAA)
        return false;

    mbrPartitionEntry_t *partition = (mbrPartitionEntry_t *) (sector + 446);

    for (int i = 0; i < 4; i++) {
        if (
            partition[i].lbaBegin > 0
            && (
                partition[i].type == MBR_PARTITION_TYPE_FAT32
                || partition[i].type == MBR_PARTITION_TYPE_FAT32_LBA
                || partition[i].type == MBR_PARTITION_TYPE_FAT16
                || partition[i].type == MBR_PARTITION_TYPE_FAT16_LBA
            )
        ) {
            afatfs.partitionStartSector = partition[i].lbaBegin;

            return true;
        }
    }

    return false;
}

static bool afatfs_parseVolumeID(const uint8_t *sector)
{
    fatVolumeID_t *volume = (fatVolumeID_t *) sector;

    afatfs.filesystemType = FAT_FILESYSTEM_TYPE_INVALID;

    if (volume->bytesPerSector != AFATFS_SECTOR_SIZE || volume->numFATs != AFATFS_NUM_FATS
            || sector[510] != FAT_VOLUME_ID_SIGNATURE_1 || sector[511] != FAT_VOLUME_ID_SIGNATURE_2) {
        return false;
    }

    afatfs.fatStartSector = afatfs.partitionStartSector + volume->reservedSectorCount;

    afatfs.sectorsPerCluster = volume->sectorsPerCluster;
    if (afatfs.sectorsPerCluster < 1 || afatfs.sectorsPerCluster > 128 || !isPowerOfTwo(afatfs.sectorsPerCluster)) {
        return false;
    }

    afatfs.byteInClusterMask = AFATFS_SECTOR_SIZE * afatfs.sectorsPerCluster - 1;

    afatfs.fatSectors = volume->FATSize16 != 0 ? volume->FATSize16 : volume->fatDescriptor.fat32.FATSize32;

    // Always zero on FAT32 since rootEntryCount is always zero (this is non-zero on FAT16)
    afatfs.rootDirectorySectors = ((volume->rootEntryCount * FAT_DIRECTORY_ENTRY_SIZE) + (volume->bytesPerSector - 1)) / volume->bytesPerSector;
    uint32_t totalSectors = volume->totalSectors16 != 0 ? volume->totalSectors16 : volume->totalSectors32;
    uint32_t dataSectors = totalSectors - (volume->reservedSectorCount + (AFATFS_NUM_FATS * afatfs.fatSectors) + afatfs.rootDirectorySectors);

    afatfs.numClusters = dataSectors / volume->sectorsPerCluster;

    if (afatfs.numClusters <= FAT12_MAX_CLUSTERS) {
        afatfs.filesystemType = FAT_FILESYSTEM_TYPE_FAT12;

        return false; // FAT12 is not a supported filesystem
    } else if (afatfs.numClusters <= FAT16_MAX_CLUSTERS) {
        afatfs.filesystemType = FAT_FILESYSTEM_TYPE_FAT16;
    } else {
        afatfs.filesystemType = FAT_FILESYSTEM_TYPE_FAT32;
    }

    if (afatfs.filesystemType == FAT_FILESYSTEM_TYPE_FAT32) {
        afatfs.rootDirectoryCluster = volume->fatDescriptor.fat32.rootCluster;
    } else {
        // FAT16 doesn't store the root directory in clusters
        afatfs.rootDirectoryCluster = 0;
    }

    uint32_t endOfFATs = afatfs.fatStartSector + AFATFS_NUM_FATS * afatfs.fatSectors;

    afatfs.clusterStartSector = endOfFATs + afatfs.rootDirectorySectors;

    return true;
}

/**
 * Get the position of the FAT entry for the cluster with the given number.
 */
static void afatfs_getFATPositionForCluster(uint32_t cluster, uint32_t *fatSectorIndex, uint32_t *fatSectorEntryIndex)
{
    if (afatfs.filesystemType == FAT_FILESYSTEM_TYPE_FAT16) {
        uint32_t entriesPerFATSector = AFATFS_SECTOR_SIZE / sizeof(uint16_t);

        *fatSectorIndex = cluster / entriesPerFATSector;
        *fatSectorEntryIndex = cluster & (entriesPerFATSector - 1);
    } else {
        uint32_t entriesPerFATSector = AFATFS_SECTOR_SIZE / sizeof(uint32_t);

        *fatSectorIndex = fat32_decodeClusterNumber(cluster) / entriesPerFATSector;
        *fatSectorEntryIndex = cluster & (entriesPerFATSector - 1);
    }
}

static bool afatfs_FATIsEndOfChainMarker(uint32_t clusterNumber)
{
    if (afatfs.filesystemType == FAT_FILESYSTEM_TYPE_FAT32) {
        return fat32_isEndOfChainMarker(clusterNumber);
    } else {
        return fat16_isEndOfChainMarker(clusterNumber);
    }
}

/**
 * Look up the FAT to find out which cluster follows the one with the given number and store it into *nextCluster.
 *
 * Use fat_isFreeSpace() and fat_isEndOfChainMarker() on nextCluster to distinguish those special values from regular
 * cluster numbers.
 *
 * Note that if you're trying to find the next cluster of a file, you should be calling afatfs_fileGetNextCluster()
 * instead, as that one supports contiguous freefile-based files (which needn't consult the FAT).
 *
 * Returns:
 *     AFATFS_OPERATION_IN_PROGRESS - FS is busy right now, call again later
 *     AFATFS_OPERATION_SUCCESS     - *nextCluster is set to the next cluster number
 */
static afatfsOperationStatus_e afatfs_FATGetNextCluster(int fatIndex, uint32_t cluster, uint32_t *nextCluster)
{
    uint32_t fatSectorIndex, fatSectorEntryIndex;
    afatfsFATSector_t sector;

    afatfs_getFATPositionForCluster(cluster, &fatSectorIndex, &fatSectorEntryIndex);

    afatfsOperationStatus_e result = afatfs_cacheSector(afatfs_fatSectorToPhysical(fatIndex, fatSectorIndex), &sector.bytes, AFATFS_CACHE_READ, 0);

    if (result == AFATFS_OPERATION_SUCCESS) {
        if (afatfs.filesystemType == FAT_FILESYSTEM_TYPE_FAT16) {
            *nextCluster = sector.fat16[fatSectorEntryIndex];
        } else {
            *nextCluster = fat32_decodeClusterNumber(sector.fat32[fatSectorEntryIndex]);
        }
    }

    return result;
}

/**
 * Set the cluster number that follows the given cluster. Pass 0xFFFFFFFF for nextCluster to terminate the FAT chain.
 *
 * Returns:
 *     AFATFS_OPERATION_SUCCESS     - On success
 *     AFATFS_OPERATION_IN_PROGRESS - Card is busy, call again later
 *     AFATFS_OPERATION_FAILURE     - When the filesystem encounters a fatal error
 */
static afatfsOperationStatus_e afatfs_FATSetNextCluster(uint32_t startCluster, uint32_t nextCluster)
{
    afatfsFATSector_t sector;
    uint32_t fatSectorIndex, fatSectorEntryIndex, fatPhysicalSector;
    afatfsOperationStatus_e result;

#ifdef AFATFS_DEBUG
    afatfs_assert(startCluster >= FAT_SMALLEST_LEGAL_CLUSTER_NUMBER);
#endif

    afatfs_getFATPositionForCluster(startCluster, &fatSectorIndex, &fatSectorEntryIndex);

    fatPhysicalSector = afatfs_fatSectorToPhysical(0, fatSectorIndex);

    result = afatfs_cacheSector(fatPhysicalSector, &sector.bytes, AFATFS_CACHE_READ | AFATFS_CACHE_WRITE, 0);

    if (result == AFATFS_OPERATION_SUCCESS) {
        if (afatfs.filesystemType == FAT_FILESYSTEM_TYPE_FAT16) {
            sector.fat16[fatSectorEntryIndex] = nextCluster;
        } else {
            sector.fat32[fatSectorEntryIndex] = nextCluster;
        }
    }

    return result;
}

/**
 * Bring the logical filesize up to date with the current cursor position.
 */
static void afatfs_fileUpdateFilesize(afatfsFile_t *file)
{
    file->logicalSize = MAX(file->logicalSize, file->cursorOffset);
}

static void afatfs_fileUnlockCacheSector(afatfsFilePtr_t file)
{
    if (file->writeLockedCacheIndex != -1) {
        afatfs.cacheDescriptor[file->writeLockedCacheIndex].locked = 0;
        file->writeLockedCacheIndex = -1;
    }
    if (file->readRetainCacheIndex != -1) {
        afatfs.cacheDescriptor[file->readRetainCacheIndex].retainCount = MAX((int) afatfs.cacheDescriptor[file->readRetainCacheIndex].retainCount - 1, 0);
        file->readRetainCacheIndex = -1;
    }
}

/**
 * Starting from and including the given cluster number, find the number of the first cluster which matches the given
 * condition.
 *
 * searchLimit - Last cluster to examine (exclusive). To search the entire volume, pass:
 *                   afatfs.numClusters + FAT_SMALLEST_LEGAL_CLUSTER_NUMBER
 *
 * Condition:
 *     CLUSTER_SEARCH_FREE_AT_BEGINNING_OF_FAT_SECTOR - Find a cluster marked as free in the FAT which lies at the
 *         beginning of its FAT sector. The passed initial search 'cluster' must correspond to the first entry of a FAT sector.
 *     CLUSTER_SEARCH_FREE            - Find a cluster marked as free in the FAT
 *     CLUSTER_SEARCH_OCCUPIED        - Find a cluster marked as occupied in the FAT.
 *
 * Returns:
 *     AFATFS_FIND_CLUSTER_FOUND       - A cluster matching the criteria was found and stored in *cluster
 *     AFATFS_FIND_CLUSTER_IN_PROGRESS - The search is not over, call this routine again later with the updated *cluster value to resume
 *     AFATFS_FIND_CLUSTER_FATAL       - An unexpected read error occurred, the volume should be abandoned
 *     AFATFS_FIND_CLUSTER_NOT_FOUND   - The entire device was searched without finding a suitable cluster (the
 *                                       *cluster points to just beyond the final cluster).
 */
static afatfsFindClusterStatus_e afatfs_findClusterWithCondition(afatfsClusterSearchCondition_e condition, uint32_t *cluster, uint32_t searchLimit)
{
    afatfsFATSector_t sector;
    uint32_t fatSectorIndex, fatSectorEntryIndex;

    uint32_t fatEntriesPerSector = afatfs_fatEntriesPerSector();
    bool lookingForFree = condition == CLUSTER_SEARCH_FREE_AT_BEGINNING_OF_FAT_SECTOR || condition == CLUSTER_SEARCH_FREE;

    int jump;

    // Get the FAT entry which corresponds to this cluster so we can begin our search there
    afatfs_getFATPositionForCluster(*cluster, &fatSectorIndex, &fatSectorEntryIndex);

    switch (condition) {
        case CLUSTER_SEARCH_FREE_AT_BEGINNING_OF_FAT_SECTOR:
            jump = fatEntriesPerSector;

            // We're supposed to call this routine with the cluster properly aligned
            if (!afatfs_assert(fatSectorEntryIndex == 0)) {
                return AFATFS_FIND_CLUSTER_FATAL;
            }
        break;
        case CLUSTER_SEARCH_OCCUPIED:
        case CLUSTER_SEARCH_FREE:
            jump = 1;
        break;
        default:
            afatfs_assert(false);
            return AFATFS_FIND_CLUSTER_FATAL;
    }

    while (*cluster < searchLimit) {

#ifdef AFATFS_USE_FREEFILE
        // If we're looking inside the freefile, we won't find any free clusters! Skip it!
        if (afatfs.freeFile.logicalSize > 0 && *cluster == afatfs.freeFile.firstCluster) {
            *cluster += (afatfs.freeFile.logicalSize + afatfs_clusterSize() - 1) / afatfs_clusterSize();

            // Maintain alignment
            *cluster = roundUpTo(*cluster, jump);
            continue; // Go back to check that the new cluster number is within the volume
        }
#endif

        afatfsOperationStatus_e status = afatfs_cacheSector(afatfs_fatSectorToPhysical(0, fatSectorIndex), &sector.bytes, AFATFS_CACHE_READ | AFATFS_CACHE_DISCARDABLE, 0);

        switch (status) {
            case AFATFS_OPERATION_SUCCESS:
                do {
                    uint32_t clusterNumber;

                    switch (afatfs.filesystemType) {
                        case FAT_FILESYSTEM_TYPE_FAT16:
                            clusterNumber = sector.fat16[fatSectorEntryIndex];
                        break;
                        case FAT_FILESYSTEM_TYPE_FAT32:
                            clusterNumber = fat32_decodeClusterNumber(sector.fat32[fatSectorEntryIndex]);
                        break;
                        default:
                            return AFATFS_FIND_CLUSTER_FATAL;
                    }

                    if (fat_isFreeSpace(clusterNumber) == lookingForFree) {
                        /*
                         * The final FAT sector may have fewer than fatEntriesPerSector entries in it, so we need to
                         * check the cluster number is valid here before we report a bogus success!
                         */
                        if (*cluster < searchLimit) {
                            return AFATFS_FIND_CLUSTER_FOUND;
                        } else {
                            *cluster = searchLimit;
                            return AFATFS_FIND_CLUSTER_NOT_FOUND;
                        }
                    }

                    (*cluster) += jump;
                    fatSectorEntryIndex += jump;
                } while (fatSectorEntryIndex < fatEntriesPerSector);

                // Move on to the next FAT sector
                fatSectorIndex++;
                fatSectorEntryIndex = 0;
            break;
            case AFATFS_OPERATION_FAILURE:
                return AFATFS_FIND_CLUSTER_FATAL;
            break;
            case AFATFS_OPERATION_IN_PROGRESS:
                return AFATFS_FIND_CLUSTER_IN_PROGRESS;
            break;
        }
    }

    // We looked at every available cluster and didn't find one matching the condition
    *cluster = searchLimit;
    return AFATFS_FIND_CLUSTER_NOT_FOUND;
}

/**
 * Get the cluster that follows the currentCluster in the FAT chain for the given file.
 *
 * Returns:
 *     AFATFS_OPERATION_IN_PROGRESS - FS is busy right now, call again later
 *     AFATFS_OPERATION_SUCCESS     - *nextCluster is set to the next cluster number
 */
static afatfsOperationStatus_e afatfs_fileGetNextCluster(afatfsFilePtr_t file, uint32_t currentCluster, uint32_t *nextCluster)
{
#ifndef AFATFS_USE_FREEFILE
    (void) file;
#else
    if ((file->mode & AFATFS_FILE_MODE_CONTIGUOUS) != 0) {
        uint32_t freeFileStart = afatfs.freeFile.firstCluster;

        afatfs_assert(currentCluster + 1 <= freeFileStart);

        // Would the next cluster lie outside the allocated file? (i.e. beyond the end of the file into the start of the freefile)
        if (currentCluster + 1 == freeFileStart) {
            *nextCluster = 0;
        } else {
            *nextCluster = currentCluster + 1;
        }

        return AFATFS_OPERATION_SUCCESS;
    } else
#endif
    {
        return afatfs_FATGetNextCluster(0, currentCluster, nextCluster);
    }
}

#ifdef AFATFS_USE_FREEFILE

/**
 * Update the FAT to fill the contiguous series of clusters with indexes [*startCluster...endCluster) with the
 * specified pattern.
 *
 * AFATFS_FAT_PATTERN_TERMINATED_CHAIN - Chain the clusters together in linear sequence and terminate the final cluster
 * AFATFS_FAT_PATTERN_CHAIN            - Chain the clusters together without terminating the final entry
 * AFATFS_FAT_PATTERN_FREE             - Mark the clusters as free space
 *
 * Returns -
 *     AFATFS_OPERATION_SUCCESS        - When the entire chain has been written
 *     AFATFS_OPERATION_IN_PROGRESS    - Call again later with the updated *startCluster value in order to resume writing.
 */
static afatfsOperationStatus_e afatfs_FATFillWithPattern(afatfsFATPattern_e pattern, uint32_t *startCluster, uint32_t endCluster)
{
    afatfsFATSector_t sector;
    uint32_t fatSectorIndex, firstEntryIndex, fatPhysicalSector;
    uint8_t fatEntrySize;
    uint32_t nextCluster;
    afatfsOperationStatus_e result;
    uint32_t eraseSectorCount;

    // Find the position of the initial cluster to begin our fill
    afatfs_getFATPositionForCluster(*startCluster, &fatSectorIndex, &firstEntryIndex);

    fatPhysicalSector = afatfs_fatSectorToPhysical(0, fatSectorIndex);

    // How many consecutive FAT sectors will we be overwriting?
    eraseSectorCount = (endCluster - *startCluster + firstEntryIndex + afatfs_fatEntriesPerSector() - 1) / afatfs_fatEntriesPerSector();

    while (*startCluster < endCluster) {
        // The last entry we will fill inside this sector (exclusive):
        uint32_t lastEntryIndex = MIN(firstEntryIndex + (endCluster - *startCluster), afatfs_fatEntriesPerSector());

        uint8_t cacheFlags = AFATFS_CACHE_WRITE | AFATFS_CACHE_DISCARDABLE;

        if (firstEntryIndex > 0 || lastEntryIndex < afatfs_fatEntriesPerSector()) {
            // We're not overwriting the entire FAT sector so we must read the existing contents
            cacheFlags |= AFATFS_CACHE_READ;
        }

        result = afatfs_cacheSector(fatPhysicalSector, &sector.bytes, cacheFlags, eraseSectorCount);

        if (result != AFATFS_OPERATION_SUCCESS) {
            return result;
        }

#ifdef AFATFS_DEBUG_VERBOSE
        if (pattern == AFATFS_FAT_PATTERN_FREE) {
            fprintf(stderr, "Marking cluster %u to %u as free in FAT sector %u...\n", *startCluster, endCluster, fatPhysicalSector);
        } else {
            fprintf(stderr, "Writing FAT chain from cluster %u to %u in FAT sector %u...\n", *startCluster, endCluster, fatPhysicalSector);
        }
#endif

        switch (pattern) {
            case AFATFS_FAT_PATTERN_TERMINATED_CHAIN:
            case AFATFS_FAT_PATTERN_UNTERMINATED_CHAIN:
                nextCluster = *startCluster + 1;
                // Write all the "next cluster" pointers
                if (afatfs.filesystemType == FAT_FILESYSTEM_TYPE_FAT16) {
                    for (uint32_t i = firstEntryIndex; i < lastEntryIndex; i++, nextCluster++) {
                        sector.fat16[i] = nextCluster;
                    }
                } else {
                    for (uint32_t i = firstEntryIndex; i < lastEntryIndex; i++, nextCluster++) {
                        sector.fat32[i] = nextCluster;
                    }
                }

                *startCluster += lastEntryIndex - firstEntryIndex;

                if (pattern == AFATFS_FAT_PATTERN_TERMINATED_CHAIN && *startCluster == endCluster) {
                    // We completed the chain! Overwrite the last entry we wrote with the terminator for the end of the chain
                    if (afatfs.filesystemType == FAT_FILESYSTEM_TYPE_FAT16) {
                        sector.fat16[lastEntryIndex - 1] = 0xFFFF;
                    } else {
                        sector.fat32[lastEntryIndex - 1] = 0xFFFFFFFF;
                    }
                    break;
                }
            break;
            case AFATFS_FAT_PATTERN_FREE:
                fatEntrySize = afatfs.filesystemType == FAT_FILESYSTEM_TYPE_FAT16 ? sizeof(uint16_t) : sizeof(uint32_t);

                memset(sector.bytes + firstEntryIndex * fatEntrySize, 0, (lastEntryIndex - firstEntryIndex) * fatEntrySize);

                *startCluster += lastEntryIndex - firstEntryIndex;
            break;
        }

        fatPhysicalSector++;
        eraseSectorCount--;
        firstEntryIndex = 0;
    }

    return AFATFS_OPERATION_SUCCESS;
}

#endif

/**
 * Write the directory entry for the file into its `directoryEntryPos` position in its containing directory.
 *
 * mode:
 *     AFATFS_SAVE_DIRECTORY_NORMAL    - Store the file's physical size, not the logical size, in the directory entry
 *     AFATFS_SAVE_DIRECTORY_FOR_CLOSE - We're done extending the file so we can write the logical size now.
 *     AFATFS_SAVE_DIRECTORY_DELETED   - Mark the directory entry as deleted
 *
 * Returns:
 *     AFATFS_OPERATION_SUCCESS - The directory entry has been stored into the directory sector in cache.
 *     AFATFS_OPERATION_IN_PROGRESS - Cache is too busy, retry later
 *     AFATFS_OPERATION_FAILURE - If the filesystem enters the fatal state
 */
static afatfsOperationStatus_e afatfs_saveDirectoryEntry(afatfsFilePtr_t file, afatfsSaveDirectoryEntryMode_e mode)
{
    uint8_t *sector;
    afatfsOperationStatus_e result;

    if (file->directoryEntryPos.sectorNumberPhysical == 0) {
        return AFATFS_OPERATION_SUCCESS; // Root directories don't have a directory entry
    }

    result = afatfs_cacheSector(file->directoryEntryPos.sectorNumberPhysical, &sector, AFATFS_CACHE_READ | AFATFS_CACHE_WRITE, 0);

#ifdef AFATFS_DEBUG_VERBOSE
    fprintf(stderr, "Saving directory entry to sector %u...\n", file->directoryEntryPos.sectorNumberPhysical);
#endif

    if (result == AFATFS_OPERATION_SUCCESS) {
        if (afatfs_assert(file->directoryEntryPos.entryIndex >= 0)) {
            fatDirectoryEntry_t *entry = (fatDirectoryEntry_t *) sector + file->directoryEntryPos.entryIndex;

            switch (mode) {
               case AFATFS_SAVE_DIRECTORY_NORMAL:
                   /* We exaggerate the length of the written file so that if power is lost, the end of the file will
                    * still be readable (though the very tail of the file will be uninitialized data).
                    *
                    * This way we can avoid updating the directory entry too many times during fwrites() on the file.
                    */
                   entry->fileSize = file->physicalSize;
               break;
               case AFATFS_SAVE_DIRECTORY_DELETED:
                   entry->filename[0] = FAT_DELETED_FILE_MARKER;
                   FALLTHROUGH;

               case AFATFS_SAVE_DIRECTORY_FOR_CLOSE:
                   // We write the true length of the file on close.
                   entry->fileSize = file->logicalSize;
            }

            // (sub)directories don't store a filesize in their directory entry:
            if (file->type == AFATFS_FILE_TYPE_DIRECTORY) {
                entry->fileSize = 0;
            }

            entry->firstClusterHigh = file->firstCluster >> 16;
            entry->firstClusterLow = file->firstCluster & 0xFFFF;
        } else {
            return AFATFS_OPERATION_FAILURE;
        }
    }

    return result;
}

/**
 * Attempt to add a free cluster to the end of the given file. If the file was previously empty, the directory entry
 * is updated to point to the new cluster.
 *
 * Returns:
 *     AFATFS_OPERATION_SUCCESS     - The cluster has been appended
 *     AFATFS_OPERATION_IN_PROGRESS - Cache was busy, so call again later to continue
 *     AFATFS_OPERATION_FAILURE     - Cluster could not be appended because the filesystem ran out of space
 *                                    (afatfs.filesystemFull is set to true)
 *
 * If the file's operation was AFATFS_FILE_OPERATION_APPEND_FREE_CLUSTER, the file operation is cleared upon completion,
 * otherwise it is left alone so that this operation can be called as a sub-operation of some other operation on the
 * file.
 */
static afatfsOperationStatus_e afatfs_appendRegularFreeClusterContinue(afatfsFile_t *file)
{
    afatfsAppendFreeCluster_t *opState = &file->operation.state.appendFreeCluster;
    afatfsOperationStatus_e status;

    doMore:

    switch (opState->phase) {
        case AFATFS_APPEND_FREE_CLUSTER_PHASE_FIND_FREESPACE:
            switch (afatfs_findClusterWithCondition(CLUSTER_SEARCH_FREE, &opState->searchCluster, afatfs.numClusters + FAT_SMALLEST_LEGAL_CLUSTER_NUMBER)) {
                case AFATFS_FIND_CLUSTER_FOUND:
                    afatfs.lastClusterAllocated = opState->searchCluster;

                    // Make the cluster available for us to write in
                    file->cursorCluster = opState->searchCluster;
                    file->physicalSize += afatfs_clusterSize();

                    if (opState->previousCluster == 0) {
                        // This is the new first cluster in the file
                        file->firstCluster = opState->searchCluster;
                    }

                    opState->phase = AFATFS_APPEND_FREE_CLUSTER_PHASE_UPDATE_FAT1;
                    goto doMore;
                break;
                case AFATFS_FIND_CLUSTER_FATAL:
                case AFATFS_FIND_CLUSTER_NOT_FOUND:
                    // We couldn't find an empty cluster to append to the file
                    opState->phase = AFATFS_APPEND_FREE_CLUSTER_PHASE_FAILURE;
                    goto doMore;
                break;
                case AFATFS_FIND_CLUSTER_IN_PROGRESS:
                break;
            }
        break;
        case AFATFS_APPEND_FREE_CLUSTER_PHASE_UPDATE_FAT1:
            // Terminate the new cluster
            status = afatfs_FATSetNextCluster(opState->searchCluster, 0xFFFFFFFF);

            if (status == AFATFS_OPERATION_SUCCESS) {
                if (opState->previousCluster) {
                    opState->phase = AFATFS_APPEND_FREE_CLUSTER_PHASE_UPDATE_FAT2;
                } else {
                    opState->phase = AFATFS_APPEND_FREE_CLUSTER_PHASE_UPDATE_FILE_DIRECTORY;
                }

                goto doMore;
            }
        break;
        case AFATFS_APPEND_FREE_CLUSTER_PHASE_UPDATE_FAT2:
            // Add the new cluster to the pre-existing chain
            status = afatfs_FATSetNextCluster(opState->previousCluster, opState->searchCluster);

            if (status == AFATFS_OPERATION_SUCCESS) {
                opState->phase = AFATFS_APPEND_FREE_CLUSTER_PHASE_UPDATE_FILE_DIRECTORY;
                goto doMore;
            }
        break;
        case AFATFS_APPEND_FREE_CLUSTER_PHASE_UPDATE_FILE_DIRECTORY:
            if (afatfs_saveDirectoryEntry(file, AFATFS_SAVE_DIRECTORY_NORMAL) == AFATFS_OPERATION_SUCCESS) {
                opState->phase = AFATFS_APPEND_FREE_CLUSTER_PHASE_COMPLETE;
                goto doMore;
            }
        break;
        case AFATFS_APPEND_FREE_CLUSTER_PHASE_COMPLETE:
            if (file->operation.operation == AFATFS_FILE_OPERATION_APPEND_FREE_CLUSTER) {
                file->operation.operation = AFATFS_FILE_OPERATION_NONE;
            }

            return AFATFS_OPERATION_SUCCESS;
        break;
        case AFATFS_APPEND_FREE_CLUSTER_PHASE_FAILURE:
            if (file->operation.operation == AFATFS_FILE_OPERATION_APPEND_FREE_CLUSTER) {
                file->operation.operation = AFATFS_FILE_OPERATION_NONE;
            }

            afatfs.filesystemFull = true;
            return AFATFS_OPERATION_FAILURE;
        break;
    }

    return AFATFS_OPERATION_IN_PROGRESS;
}

static void afatfs_appendRegularFreeClusterInitOperationState(afatfsAppendFreeCluster_t *state, uint32_t previousCluster)
{
    state->phase = AFATFS_APPEND_FREE_CLUSTER_PHASE_INITIAL;
    state->previousCluster = previousCluster;
    state->searchCluster = afatfs.lastClusterAllocated;
}

/**
 * Queue up an operation to append a free cluster to the file and update the file's cursorCluster to point to it.
 *
 * You must seek to the end of the file first, so file.cursorCluster will be 0 for the first call, and
 * `file.cursorPreviousCluster` will be the cluster to append after.
 *
 * Note that the cursorCluster will be updated before this operation is completely finished (i.e. before the FAT is
 * updated) but you can go ahead and write to it before the operation succeeds.
 *
 * Returns:
 *     AFATFS_OPERATION_SUCCESS     - The append completed successfully
 *     AFATFS_OPERATION_IN_PROGRESS - The operation was queued on the file and will complete later
 *     AFATFS_OPERATION_FAILURE     - Operation could not be queued or append failed, check afatfs.fileSystemFull
 */
static afatfsOperationStatus_e afatfs_appendRegularFreeCluster(afatfsFilePtr_t file)
{
    if (file->operation.operation == AFATFS_FILE_OPERATION_APPEND_FREE_CLUSTER)
        return AFATFS_OPERATION_IN_PROGRESS;

    if (afatfs.filesystemFull || afatfs_fileIsBusy(file)) {
        return AFATFS_OPERATION_FAILURE;
    }

    file->operation.operation = AFATFS_FILE_OPERATION_APPEND_FREE_CLUSTER;

    afatfs_appendRegularFreeClusterInitOperationState(&file->operation.state.appendFreeCluster, file->cursorPreviousCluster);

    return afatfs_appendRegularFreeClusterContinue(file);
}

#ifdef AFATFS_USE_FREEFILE

/**
 * Size of a AFATFS supercluster in bytes
 */
ONLY_EXPOSE_FOR_TESTING
uint32_t afatfs_superClusterSize(void)
{
    return afatfs_fatEntriesPerSector() * afatfs_clusterSize();
}

/**
 * Continue to attempt to add a supercluster to the end of the given file.
 *
 * If the file operation was set to AFATFS_FILE_OPERATION_APPEND_SUPERCLUSTER and the operation completes, the file's
 * operation is cleared.
 *
 * Returns:
 *     AFATFS_OPERATION_SUCCESS     - On completion
 *     AFATFS_OPERATION_IN_PROGRESS - Operation still in progress
 */
static afatfsOperationStatus_e afatfs_appendSuperclusterContinue(afatfsFile_t *file)
{
    afatfsAppendSupercluster_t *opState = &file->operation.state.appendSupercluster;

    afatfsOperationStatus_e status = AFATFS_OPERATION_FAILURE;

    doMore:
    switch (opState->phase) {
        case AFATFS_APPEND_SUPERCLUSTER_PHASE_INIT:
            // Our file steals the first cluster of the freefile

            // We can go ahead and write to that space before the FAT and directory are updated
            file->cursorCluster = afatfs.freeFile.firstCluster;
            file->physicalSize += afatfs_superClusterSize();

            /* Remove the first supercluster from the freefile
             *
             * Even if the freefile becomes empty, we still don't set its first cluster to zero. This is so that
             * afatfs_fileGetNextCluster() can tell where a contiguous file ends (at the start of the freefile).
             *
             * Note that normally the freefile can't become empty because it is allocated as a non-integer number
             * of superclusters to avoid precisely this situation.
             */
            afatfs.freeFile.firstCluster += afatfs_fatEntriesPerSector();
            afatfs.freeFile.logicalSize -= afatfs_superClusterSize();
            afatfs.freeFile.physicalSize -= afatfs_superClusterSize();

            // The new supercluster needs to have its clusters chained contiguously and marked with a terminator at the end
            opState->fatRewriteStartCluster = file->cursorCluster;
            opState->fatRewriteEndCluster = opState->fatRewriteStartCluster + afatfs_fatEntriesPerSector();

            if (opState->previousCluster == 0) {
                // This is the new first cluster in the file so we need to update the directory entry
                file->firstCluster = file->cursorCluster;
            } else {
                /*
                 * We also need to update the FAT of the supercluster that used to end the file so that it no longer
                 * terminates there
                 */
                opState->fatRewriteStartCluster -= afatfs_fatEntriesPerSector();
            }

            opState->phase = AFATFS_APPEND_SUPERCLUSTER_PHASE_UPDATE_FREEFILE_DIRECTORY;
            goto doMore;
        break;
        case AFATFS_APPEND_SUPERCLUSTER_PHASE_UPDATE_FREEFILE_DIRECTORY:
            // First update the freefile's directory entry to remove the first supercluster so we don't risk cross-linking the file
            status = afatfs_saveDirectoryEntry(&afatfs.freeFile, AFATFS_SAVE_DIRECTORY_NORMAL);

            if (status == AFATFS_OPERATION_SUCCESS) {
                opState->phase = AFATFS_APPEND_SUPERCLUSTER_PHASE_UPDATE_FAT;
                goto doMore;
            }
        break;
        case AFATFS_APPEND_SUPERCLUSTER_PHASE_UPDATE_FAT:
            status = afatfs_FATFillWithPattern(AFATFS_FAT_PATTERN_TERMINATED_CHAIN, &opState->fatRewriteStartCluster, opState->fatRewriteEndCluster);

            if (status == AFATFS_OPERATION_SUCCESS) {
                opState->phase = AFATFS_APPEND_SUPERCLUSTER_PHASE_UPDATE_FILE_DIRECTORY;
                goto doMore;
            }
        break;
        case AFATFS_APPEND_SUPERCLUSTER_PHASE_UPDATE_FILE_DIRECTORY:
            // Update the fileSize/firstCluster in the directory entry for the file
            status = afatfs_saveDirectoryEntry(file, AFATFS_SAVE_DIRECTORY_NORMAL);
        break;
    }

    if ((status == AFATFS_OPERATION_FAILURE || status == AFATFS_OPERATION_SUCCESS) && file->operation.operation == AFATFS_FILE_OPERATION_APPEND_SUPERCLUSTER) {
        file->operation.operation = AFATFS_FILE_OPERATION_NONE;
    }

    return status;
}

/**
 * Attempt to queue up an operation to append the first supercluster of the freefile to the given `file` (file's cursor
 * must be at end-of-file).
 *
 * The new cluster number will be set into the file's cursorCluster.
 *
 * Returns:
 *     AFATFS_OPERATION_SUCCESS     - The append completed successfully and the file's cursorCluster has been updated
 *     AFATFS_OPERATION_IN_PROGRESS - The operation was queued on the file and will complete later, or there is already an
 *                                    append in progress.
 *     AFATFS_OPERATION_FAILURE     - Operation could not be queued (file was busy) or append failed (filesystem is full).
 *                                    Check afatfs.fileSystemFull
 */
static afatfsOperationStatus_e afatfs_appendSupercluster(afatfsFilePtr_t file)
{
    uint32_t superClusterSize = afatfs_superClusterSize();

    if (file->operation.operation == AFATFS_FILE_OPERATION_APPEND_SUPERCLUSTER) {
        return AFATFS_OPERATION_IN_PROGRESS;
    }

    if (afatfs.freeFile.logicalSize < superClusterSize) {
        afatfs.filesystemFull = true;
    }

    if (afatfs.filesystemFull || afatfs_fileIsBusy(file)) {
        return AFATFS_OPERATION_FAILURE;
    }

    afatfsAppendSupercluster_t *opState = &file->operation.state.appendSupercluster;

    file->operation.operation = AFATFS_FILE_OPERATION_APPEND_SUPERCLUSTER;
    opState->phase = AFATFS_APPEND_SUPERCLUSTER_PHASE_INIT;
    opState->previousCluster = file->cursorPreviousCluster;

    return afatfs_appendSuperclusterContinue(file);
}

#endif

/**
 * Queue an operation to add a cluster of free space to the end of the file. Must be called when the file's cursor
 * is beyond the last allocated cluster.
 */
static afatfsOperationStatus_e afatfs_appendFreeCluster(afatfsFilePtr_t file)
{
    afatfsOperationStatus_e status;

#ifdef AFATFS_USE_FREEFILE
    if ((file->mode & AFATFS_FILE_MODE_CONTIGUOUS) != 0) {
        // Steal the first cluster from the beginning of the freefile if we can
        status = afatfs_appendSupercluster(file);
    } else
#endif
    {
        status = afatfs_appendRegularFreeCluster(file);
    }

    return status;
}

/**
 * Returns true if the file's cursor is sitting beyond the end of the last allocated cluster (i.e. the logical fileSize
 * is not checked).
 */
static bool afatfs_isEndOfAllocatedFile(afatfsFilePtr_t file)
{
    if (file->type == AFATFS_FILE_TYPE_FAT16_ROOT_DIRECTORY) {
        return file->cursorOffset >= AFATFS_SECTOR_SIZE * afatfs.rootDirectorySectors;
    } else {
        return file->cursorCluster == 0 || afatfs_FATIsEndOfChainMarker(file->cursorCluster);
    }
}

/**
 * Take a lock on the sector at the current file cursor position.
 *
 * Returns a pointer to the sector buffer if successful, or NULL if at the end of file (check afatfs_isEndOfAllocatedFile())
 * or the sector has not yet been read in from disk.
 */
static uint8_t* afatfs_fileRetainCursorSectorForRead(afatfsFilePtr_t file)
{
    uint8_t *result;

    uint32_t physicalSector = afatfs_fileGetCursorPhysicalSector(file);

    /* If we've already got a locked sector then we can assume that was the same one that's at the cursor (because this
     * cache is invalidated when crossing a sector boundary)
     */
    if (file->readRetainCacheIndex != -1) {
        if (!afatfs_assert(physicalSector == afatfs.cacheDescriptor[file->readRetainCacheIndex].sectorIndex)) {
            return NULL;
        }

        result = afatfs_cacheSectorGetMemory(file->readRetainCacheIndex);
    } else {
        if (afatfs_isEndOfAllocatedFile(file)) {
            return NULL;
        }

        afatfs_assert(physicalSector > 0); // We never read the root sector using files

        afatfsOperationStatus_e status = afatfs_cacheSector(
            physicalSector,
            &result,
            AFATFS_CACHE_READ | AFATFS_CACHE_RETAIN,
            0
        );

        if (status != AFATFS_OPERATION_SUCCESS) {
            // Sector not ready for read
            return NULL;
        }

        file->readRetainCacheIndex = afatfs_getCacheDescriptorIndexForBuffer(result);
    }

    return result;
}

/**
 * Lock the sector at the file's cursor position for write, and return a reference to the memory for that sector.
 *
 * Returns NULL if the cache was too busy, try again later.
 */
static uint8_t* afatfs_fileLockCursorSectorForWrite(afatfsFilePtr_t file)
{
    afatfsOperationStatus_e status;
    uint8_t *result;
    uint32_t eraseBlockCount;

    // Do we already have a sector locked in our cache at the cursor position?
    if (file->writeLockedCacheIndex != -1) {
        uint32_t physicalSector = afatfs_fileGetCursorPhysicalSector(file);

        if (!afatfs_assert(physicalSector == afatfs.cacheDescriptor[file->writeLockedCacheIndex].sectorIndex)) {
            return NULL;
        }

        result = afatfs_cacheSectorGetMemory(file->writeLockedCacheIndex);
    } else {
        // Find / allocate a sector and lock it in the cache so we can rely on it sticking around

        // Are we at the start of an empty file or the end of a non-empty file? If so we need to add a cluster
        if (afatfs_isEndOfAllocatedFile(file) && afatfs_appendFreeCluster(file) != AFATFS_OPERATION_SUCCESS) {
            // The extension of the file is in progress so please call us again later to try again
            return NULL;
        }

        uint32_t physicalSector = afatfs_fileGetCursorPhysicalSector(file);
        uint8_t cacheFlags = AFATFS_CACHE_WRITE | AFATFS_CACHE_LOCK;
        uint32_t cursorOffsetInSector = file->cursorOffset % AFATFS_SECTOR_SIZE;
        uint32_t offsetOfStartOfSector = file->cursorOffset & ~((uint32_t) AFATFS_SECTOR_SIZE - 1);
        uint32_t offsetOfEndOfSector = offsetOfStartOfSector + AFATFS_SECTOR_SIZE;

        /*
         * If there is data before the write point in this sector, or there could be data after the write-point
         * then we need to have the original contents of the sector in the cache for us to merge into
         */
        if (
            cursorOffsetInSector > 0
            || offsetOfEndOfSector < file->logicalSize
        ) {
            cacheFlags |= AFATFS_CACHE_READ;
        }

        // In contiguous append mode, we'll pre-erase the whole supercluster
        if ((file->mode & (AFATFS_FILE_MODE_APPEND | AFATFS_FILE_MODE_CONTIGUOUS)) == (AFATFS_FILE_MODE_APPEND | AFATFS_FILE_MODE_CONTIGUOUS)) {
            uint32_t cursorOffsetInSupercluster = file->cursorOffset & (afatfs_superClusterSize() - 1);

            eraseBlockCount = afatfs_fatEntriesPerSector() * afatfs.sectorsPerCluster - cursorOffsetInSupercluster / AFATFS_SECTOR_SIZE;
        } else {
            eraseBlockCount = 0;
        }

        status = afatfs_cacheSector(
            physicalSector,
            &result,
            cacheFlags,
            eraseBlockCount
        );

        if (status != AFATFS_OPERATION_SUCCESS) {
            // Not enough cache available to accept this write / sector not ready for read
            return NULL;
        }

        file->writeLockedCacheIndex = afatfs_getCacheDescriptorIndexForBuffer(result);
    }

    return result;
}

/**
 * Attempt to seek the file pointer by the offset, relative to the current position.
 *
 * Returns true if the seek was completed, or false if you should try again later by calling this routine again (the
 * cursor is not moved and no seek operation is queued on the file for you).
 *
 * You can only seek forwards by the size of a cluster or less, or backwards to stay within the same cluster. Otherwise
 * false will always be returned (calling this routine again will never make progress on the seek).
 *
 * This amount of seek is special because we will have to wait on at most one read operation, so it's easy to make
 * the seek atomic.
 */
static bool afatfs_fseekAtomic(afatfsFilePtr_t file, int32_t offset)
{
    // Seeks within a sector
    uint32_t newSectorOffset = offset + file->cursorOffset % AFATFS_SECTOR_SIZE;

    // i.e. newSectorOffset is non-negative and smaller than AFATFS_SECTOR_SIZE, we're staying within the same sector
    if (newSectorOffset < AFATFS_SECTOR_SIZE) {
        file->cursorOffset += offset;
        return true;
    }

    // We're seeking outside the sector so unlock it if we were holding it
    afatfs_fileUnlockCacheSector(file);

    // FAT16 root directories are made up of contiguous sectors rather than clusters
    if (file->type == AFATFS_FILE_TYPE_FAT16_ROOT_DIRECTORY) {
        file->cursorOffset += offset;

        return true;
    }

    uint32_t clusterSizeBytes = afatfs_clusterSize();
    uint32_t offsetInCluster = afatfs_byteIndexInCluster(file->cursorOffset);
    uint32_t newOffsetInCluster = offsetInCluster + offset;

    afatfsOperationStatus_e status;

    if (offset > (int32_t) clusterSizeBytes || offset < -(int32_t) offsetInCluster) {
        return false;
    }

    // Are we seeking outside the cluster? If so we'll need to find out the next cluster number
    if (newOffsetInCluster >= clusterSizeBytes) {
        uint32_t nextCluster;

        status = afatfs_fileGetNextCluster(file, file->cursorCluster, &nextCluster);

        if (status == AFATFS_OPERATION_SUCCESS) {
            // Seek to the beginning of the next cluster
            uint32_t bytesToSeek = clusterSizeBytes - offsetInCluster;

            file->cursorPreviousCluster = file->cursorCluster;
            file->cursorCluster = nextCluster;
            file->cursorOffset += bytesToSeek;

            offset -= bytesToSeek;
        } else {
            // Try again later
            return false;
        }
    }

    // If we didn't already hit the end of the file, add any remaining offset needed inside the cluster
    if (!afatfs_isEndOfAllocatedFile(file)) {
        file->cursorOffset += offset;
    }

    return true;
}

/**
 * Returns true if the seek was completed, or false if it is still in progress.
 */
static bool afatfs_fseekInternalContinue(afatfsFile_t *file)
{
    afatfsSeek_t *opState = &file->operation.state.seek;
    uint32_t clusterSizeBytes = afatfs_clusterSize();
    uint32_t offsetInCluster = afatfs_byteIndexInCluster(file->cursorOffset);

    afatfsOperationStatus_e status;

    // Keep advancing the cursor cluster forwards to consume seekOffset
    while (offsetInCluster + opState->seekOffset >= clusterSizeBytes && !afatfs_isEndOfAllocatedFile(file)) {
        uint32_t nextCluster;

        status = afatfs_fileGetNextCluster(file, file->cursorCluster, &nextCluster);

        if (status == AFATFS_OPERATION_SUCCESS) {
            // Seek to the beginning of the next cluster
            uint32_t bytesToSeek = clusterSizeBytes - offsetInCluster;

            file->cursorPreviousCluster = file->cursorCluster;
            file->cursorCluster = nextCluster;

            file->cursorOffset += bytesToSeek;
            opState->seekOffset -= bytesToSeek;
            offsetInCluster = 0;
        } else {
            // Try again later
            return false;
        }
    }

    // If we didn't already hit the end of the file, add any remaining offset needed inside the cluster
    if (!afatfs_isEndOfAllocatedFile(file)) {
        file->cursorOffset += opState->seekOffset;
    }

    afatfs_fileUpdateFilesize(file); // TODO do we need this?

    file->operation.operation = AFATFS_FILE_OPERATION_NONE;

    if (opState->callback) {
        opState->callback(file);
    }

    return true;
}

/**
 * Seek the file pointer forwards by offset bytes. Calls the callback when the seek is complete.
 *
 * Will happily seek beyond the logical end of the file.
 *
 * Returns:
 *     AFATFS_OPERATION_SUCCESS     - The seek was completed immediately
 *     AFATFS_OPERATION_IN_PROGRESS - The seek was queued and will complete later
 *     AFATFS_OPERATION_FAILURE     - The seek could not be queued because the file was busy with another operation,
 *                                    try again later.
 */
static afatfsOperationStatus_e afatfs_fseekInternal(afatfsFilePtr_t file, uint32_t offset, afatfsFileCallback_t callback)
{
    // See if we can seek without queuing an operation
    if (afatfs_fseekAtomic(file, offset)) {
        if (callback) {
            callback(file);
        }

        return AFATFS_OPERATION_SUCCESS;
    } else {
        // Our operation must queue
        if (afatfs_fileIsBusy(file)) {
            return AFATFS_OPERATION_FAILURE;
        }

        afatfsSeek_t *opState = &file->operation.state.seek;

        file->operation.operation = AFATFS_FILE_OPERATION_SEEK;
        opState->callback = callback;
        opState->seekOffset = offset;

        return AFATFS_OPERATION_IN_PROGRESS;
    }
}

/**
 * Attempt to seek the file cursor from the given point (`whence`) by the given offset, just like C's fseek().
 *
 * AFATFS_SEEK_SET with offset 0 will always return AFATFS_OPERATION_SUCCESS.
 *
 * Returns:
 *     AFATFS_OPERATION_SUCCESS     - The seek was completed immediately
 *     AFATFS_OPERATION_IN_PROGRESS - The seek was queued and will complete later. Feel free to attempt read/write
 *                                    operations on the file, they'll fail until the seek is complete.
 *     AFATFS_OPERATION_FAILURE     - The seek could not be queued because the file was busy with another operation,
 *                                    try again later.
 */
afatfsOperationStatus_e afatfs_fseek(afatfsFilePtr_t file, int32_t offset, afatfsSeek_e whence)
{
    // We need an up-to-date logical filesize so we can clamp seeks to the EOF
    afatfs_fileUpdateFilesize(file);

    switch (whence) {
        case AFATFS_SEEK_CUR:
            if (offset >= 0) {
                // Only forwards seeks are supported by this routine:
                return afatfs_fseekInternal(file, MIN(file->cursorOffset + offset, file->logicalSize), NULL);
            }

            // Convert a backwards relative seek into a SEEK_SET. TODO considerable room for improvement if within the same cluster
            offset += file->cursorOffset;
        break;

        case AFATFS_SEEK_END:
            // Are we already at the right position?
            if (file->logicalSize + offset == file->cursorOffset) {
                return AFATFS_OPERATION_SUCCESS;
            }

            // Convert into a SEEK_SET
            offset += file->logicalSize;
        break;

        case AFATFS_SEEK_SET:
            FALLTHROUGH;
    }

    // Now we have a SEEK_SET with a positive offset. Begin by seeking to the start of the file
    afatfs_fileUnlockCacheSector(file);

    file->cursorPreviousCluster = 0;
    file->cursorCluster = file->firstCluster;
    file->cursorOffset = 0;

    // Then seek forwards by the offset
    return afatfs_fseekInternal(file, MIN((uint32_t) offset, file->logicalSize), NULL);
}

/**
 * Get the byte-offset of the file's cursor from the start of the file.
 *
 * Returns true on success, or false if the file is busy (try again later).
 */
bool afatfs_ftell(afatfsFilePtr_t file, uint32_t *position)
{
    if (afatfs_fileIsBusy(file)) {
        return false;
    } else {
        *position = file->cursorOffset;
        return true;
    }
}

/**
 * Attempt to advance the directory pointer `finder` to the next entry in the directory.
 *
 * Returns:
 *     AFATFS_OPERATION_SUCCESS -     A pointer to the next directory entry has been loaded into *dirEntry. If the
 *                                    directory was exhausted then *dirEntry will be set to NULL.
 *     AFATFS_OPERATION_IN_PROGRESS - The disk is busy. The pointer is not advanced, call again later to retry.
 */
afatfsOperationStatus_e afatfs_findNext(afatfsFilePtr_t directory, afatfsFinder_t *finder, fatDirectoryEntry_t **dirEntry)
{
    uint8_t *sector;

    if (finder->entryIndex == AFATFS_FILES_PER_DIRECTORY_SECTOR - 1) {
        if (afatfs_fseekAtomic(directory, AFATFS_SECTOR_SIZE)) {
            finder->entryIndex = -1;
            // Fall through to read the first entry of that new sector
        } else {
            return AFATFS_OPERATION_IN_PROGRESS;
        }
    }

    sector = afatfs_fileRetainCursorSectorForRead(directory);

    if (sector) {
        finder->entryIndex++;

        *dirEntry = (fatDirectoryEntry_t*) sector + finder->entryIndex;

        finder->sectorNumberPhysical = afatfs_fileGetCursorPhysicalSector(directory);

        return AFATFS_OPERATION_SUCCESS;
    } else {
        if (afatfs_isEndOfAllocatedFile(directory)) {
            *dirEntry = NULL;

            return AFATFS_OPERATION_SUCCESS;
        }

        return AFATFS_OPERATION_IN_PROGRESS;
    }
}

/**
 * Release resources associated with a find operation. Calling this more than once is harmless.
 */
void afatfs_findLast(afatfsFilePtr_t directory)
{
    afatfs_fileUnlockCacheSector(directory);
}

/**
 * Initialise the finder so that the first call with the directory to findNext() will return the first file in the
 * directory.
 */
void afatfs_findFirst(afatfsFilePtr_t directory, afatfsFinder_t *finder)
{
    afatfs_fseek(directory, 0, AFATFS_SEEK_SET);
    finder->entryIndex = -1;
}

static afatfsOperationStatus_e afatfs_extendSubdirectoryContinue(afatfsFile_t *directory)
{
    afatfsExtendSubdirectory_t *opState = &directory->operation.state.extendSubdirectory;
    afatfsOperationStatus_e status;
    uint8_t *sectorBuffer;
    uint32_t clusterNumber, physicalSector;
    uint16_t sectorInCluster;

    doMore:
    switch (opState->phase) {
        case AFATFS_EXTEND_SUBDIRECTORY_PHASE_ADD_FREE_CLUSTER:
            status = afatfs_appendRegularFreeClusterContinue(directory);

            if (status == AFATFS_OPERATION_SUCCESS) {
                opState->phase = AFATFS_EXTEND_SUBDIRECTORY_PHASE_WRITE_SECTORS;
                goto doMore;
            } else if (status == AFATFS_OPERATION_FAILURE) {
                opState->phase = AFATFS_EXTEND_SUBDIRECTORY_PHASE_FAILURE;
                goto doMore;
            }
        break;
        case AFATFS_EXTEND_SUBDIRECTORY_PHASE_WRITE_SECTORS:
            // Now, zero out that cluster
            afatfs_fileGetCursorClusterAndSector(directory, &clusterNumber, &sectorInCluster);
            physicalSector = afatfs_fileGetCursorPhysicalSector(directory);

            while (1) {
                status = afatfs_cacheSector(physicalSector, &sectorBuffer, AFATFS_CACHE_WRITE, 0);

                if (status != AFATFS_OPERATION_SUCCESS) {
                    return status;
                }

                memset(sectorBuffer, 0, AFATFS_SECTOR_SIZE);

                // If this is the first sector of a non-root directory, create the "." and ".." entries
                if (directory->directoryEntryPos.sectorNumberPhysical != 0 && directory->cursorOffset == 0) {
                    fatDirectoryEntry_t *dirEntries = (fatDirectoryEntry_t *) sectorBuffer;

                    memset(dirEntries[0].filename, ' ', sizeof(dirEntries[0].filename));
                    dirEntries[0].filename[0] = '.';
                    dirEntries[0].firstClusterHigh = directory->firstCluster >> 16;
                    dirEntries[0].firstClusterLow = directory->firstCluster & 0xFFFF;
                    dirEntries[0].attrib = FAT_FILE_ATTRIBUTE_DIRECTORY;

                    memset(dirEntries[1].filename, ' ', sizeof(dirEntries[1].filename));
                    dirEntries[1].filename[0] = '.';
                    dirEntries[1].filename[1] = '.';
                    dirEntries[1].firstClusterHigh = opState->parentDirectoryCluster >> 16;
                    dirEntries[1].firstClusterLow = opState->parentDirectoryCluster & 0xFFFF;
                    dirEntries[1].attrib = FAT_FILE_ATTRIBUTE_DIRECTORY;
                }

                if (sectorInCluster < afatfs.sectorsPerCluster - 1) {
                    // Move to next sector
                    afatfs_assert(afatfs_fseekAtomic(directory, AFATFS_SECTOR_SIZE));
                    sectorInCluster++;
                    physicalSector++;
                } else {
                    break;
                }
            }

            // Seek back to the beginning of the cluster
            afatfs_assert(afatfs_fseekAtomic(directory, -AFATFS_SECTOR_SIZE * (afatfs.sectorsPerCluster - 1)));
            opState->phase = AFATFS_EXTEND_SUBDIRECTORY_PHASE_SUCCESS;
            goto doMore;
        break;
        case AFATFS_EXTEND_SUBDIRECTORY_PHASE_SUCCESS:
            directory->operation.operation = AFATFS_FILE_OPERATION_NONE;

            if (opState->callback) {
                opState->callback(directory);
            }

            return AFATFS_OPERATION_SUCCESS;
        break;
        case AFATFS_EXTEND_SUBDIRECTORY_PHASE_FAILURE:
            directory->operation.operation = AFATFS_FILE_OPERATION_NONE;

            if (opState->callback) {
                opState->callback(NULL);
            }
            return AFATFS_OPERATION_FAILURE;
        break;
    }

    return AFATFS_OPERATION_IN_PROGRESS;
}

/**
 * Queue an operation to add a cluster to a sub-directory.
 *
 * Tthe new cluster is zero-filled. "." and ".." entries are added if it is the first cluster of a new subdirectory.
 *
 * The directory must not be busy, otherwise AFATFS_OPERATION_FAILURE is returned immediately.
 *
 * The directory's cursor must lie at the end of the directory file (i.e. isEndOfAllocatedFile() would return true).
 *
 * You must provide parentDirectory if this is the first extension to the subdirectory, otherwise pass NULL for that argument.
 */
static afatfsOperationStatus_e afatfs_extendSubdirectory(afatfsFile_t *directory, afatfsFilePtr_t parentDirectory, afatfsFileCallback_t callback)
{
    // FAT16 root directories cannot be extended
    if (directory->type == AFATFS_FILE_TYPE_FAT16_ROOT_DIRECTORY || afatfs_fileIsBusy(directory)) {
        return AFATFS_OPERATION_FAILURE;
    }

    /*
     * We'll assume that we're never asked to append the first cluster of a root directory, since any
     * reasonably-formatted volume should have a root!
     */
    afatfsExtendSubdirectory_t *opState = &directory->operation.state.extendSubdirectory;

    directory->operation.operation = AFATFS_FILE_OPERATION_EXTEND_SUBDIRECTORY;

    opState->phase = AFATFS_EXTEND_SUBDIRECTORY_PHASE_INITIAL;
    opState->parentDirectoryCluster = parentDirectory ? parentDirectory->firstCluster : 0;
    opState->callback = callback;

    afatfs_appendRegularFreeClusterInitOperationState(&opState->appendFreeCluster, directory->cursorPreviousCluster);

    return afatfs_extendSubdirectoryContinue(directory);
}

/**
 * Allocate space for a new directory entry to be written, store the position of that entry in the finder, and set
 * the *dirEntry pointer to point to the entry within the cached FAT sector. This pointer's lifetime is only as good
 * as the life of the cache, so don't dawdle.
 *
 * Before the first call to this function, call afatfs_findFirst() on the directory.
 *
 * The directory sector in the cache is marked as dirty, so any changes written through to the entry will be flushed out
 * in a subsequent poll cycle.
 *
 * Returns:
 *     AFATFS_OPERATION_IN_PROGRESS - Call again later to continue
 *     AFATFS_OPERATION_SUCCESS     - Entry has been inserted and *dirEntry and *finder have been updated
 *     AFATFS_OPERATION_FAILURE     - When the directory is full.
 */
static afatfsOperationStatus_e afatfs_allocateDirectoryEntry(afatfsFilePtr_t directory, fatDirectoryEntry_t **dirEntry, afatfsFinder_t *finder)
{
    afatfsOperationStatus_e result;

    if (afatfs_fileIsBusy(directory)) {
        return AFATFS_OPERATION_IN_PROGRESS;
    }

    while ((result = afatfs_findNext(directory, finder, dirEntry)) == AFATFS_OPERATION_SUCCESS) {
        if (*dirEntry) {
            if (fat_isDirectoryEntryEmpty(*dirEntry) || fat_isDirectoryEntryTerminator(*dirEntry)) {
                afatfs_cacheSectorMarkDirty(afatfs_getCacheDescriptorForBuffer((uint8_t*) *dirEntry));

                afatfs_findLast(directory);
                return AFATFS_OPERATION_SUCCESS;
            }
        } else {
            // Need to extend directory size by adding a cluster
            result = afatfs_extendSubdirectory(directory, NULL, NULL);

            if (result == AFATFS_OPERATION_SUCCESS) {
                // Continue the search in the newly-extended directory
                continue;
            } else {
                // The status (in progress or failure) of extending the directory becomes our status
                break;
            }
        }
    }

    return result;
}

/**
 * Return a pointer to a free entry in the open files table (a file whose type is "NONE"). You should initialize
 * the file afterwards with afatfs_initFileHandle().
 */
static afatfsFilePtr_t afatfs_allocateFileHandle(void)
{
    for (int i = 0; i < AFATFS_MAX_OPEN_FILES; i++) {
        if (afatfs.openFiles[i].type == AFATFS_FILE_TYPE_NONE) {
            return &afatfs.openFiles[i];
        }
    }

    return NULL;
}

/**
 * Continue the file truncation.
 *
 * When truncation finally succeeds or fails, the current operation is cleared on the file (if the current operation
 * was a truncate), then the truncate operation's callback is called. This allows the truncation to be called as a
 * sub-operation without it clearing the parent file operation.
 */
static afatfsOperationStatus_e afatfs_ftruncateContinue(afatfsFilePtr_t file, bool markDeleted)
{
    afatfsTruncateFile_t *opState = &file->operation.state.truncateFile;
    afatfsOperationStatus_e status = AFATFS_OPERATION_FAILURE;

#ifdef AFATFS_USE_FREEFILE
    uint32_t oldFreeFileStart, freeFileGrow;
#endif

    doMore:

    switch (opState->phase) {
        case AFATFS_TRUNCATE_FILE_UPDATE_DIRECTORY:
            status = afatfs_saveDirectoryEntry(file, markDeleted ? AFATFS_SAVE_DIRECTORY_DELETED : AFATFS_SAVE_DIRECTORY_NORMAL);

            if (status == AFATFS_OPERATION_SUCCESS) {
#ifdef AFATFS_USE_FREEFILE
                if (opState->endCluster) {
                    opState->phase = AFATFS_TRUNCATE_FILE_ERASE_FAT_CHAIN_CONTIGUOUS;
                } else
#endif
                {
                    opState->phase = AFATFS_TRUNCATE_FILE_ERASE_FAT_CHAIN_NORMAL;
                }
                goto doMore;
            }
        break;
#ifdef AFATFS_USE_FREEFILE
        case AFATFS_TRUNCATE_FILE_ERASE_FAT_CHAIN_CONTIGUOUS:
            // Prepare the clusters to be added back on to the beginning of the freefile
            status = afatfs_FATFillWithPattern(AFATFS_FAT_PATTERN_UNTERMINATED_CHAIN, &opState->currentCluster, opState->endCluster);

            if (status == AFATFS_OPERATION_SUCCESS) {
                opState->phase = AFATFS_TRUNCATE_FILE_PREPEND_TO_FREEFILE;
                goto doMore;
            }
        break;
        case AFATFS_TRUNCATE_FILE_PREPEND_TO_FREEFILE:
            // Note, it's okay to run this code several times:
            oldFreeFileStart = afatfs.freeFile.firstCluster;

            afatfs.freeFile.firstCluster = opState->startCluster;

            freeFileGrow = (oldFreeFileStart - opState->startCluster) * afatfs_clusterSize();

            afatfs.freeFile.logicalSize += freeFileGrow;
            afatfs.freeFile.physicalSize += freeFileGrow;

            status = afatfs_saveDirectoryEntry(&afatfs.freeFile, AFATFS_SAVE_DIRECTORY_NORMAL);
            if (status == AFATFS_OPERATION_SUCCESS) {
                opState->phase = AFATFS_TRUNCATE_FILE_SUCCESS;
                goto doMore;
            }
        break;
#endif
        case AFATFS_TRUNCATE_FILE_ERASE_FAT_CHAIN_NORMAL:
            while (!afatfs_FATIsEndOfChainMarker(opState->currentCluster)) {
                uint32_t nextCluster;

                status = afatfs_FATGetNextCluster(0, opState->currentCluster, &nextCluster);

                if (status != AFATFS_OPERATION_SUCCESS) {
                    return status;
                }

                status = afatfs_FATSetNextCluster(opState->currentCluster, 0);

                if (status != AFATFS_OPERATION_SUCCESS) {
                    return status;
                }

                opState->currentCluster = nextCluster;

                // Searches for unallocated regular clusters should be told about this free cluster now
                afatfs.lastClusterAllocated = MIN(afatfs.lastClusterAllocated, opState->currentCluster - 1);
            }

            opState->phase = AFATFS_TRUNCATE_FILE_SUCCESS;
            goto doMore;
        break;
        case AFATFS_TRUNCATE_FILE_SUCCESS:
            if (file->operation.operation == AFATFS_FILE_OPERATION_TRUNCATE) {
                file->operation.operation = AFATFS_FILE_OPERATION_NONE;
            }

            if (opState->callback) {
                opState->callback(file);
            }

            return AFATFS_OPERATION_SUCCESS;
        break;
    }

    if (status == AFATFS_OPERATION_FAILURE && file->operation.operation == AFATFS_FILE_OPERATION_TRUNCATE) {
        file->operation.operation = AFATFS_FILE_OPERATION_NONE;
    }

    return status;
}

/**
 * Queue an operation to truncate the file to zero bytes in length.
 *
 * Returns true if the operation was successfully queued or false if the file is busy (try again later).
 *
 * The callback is called once the file has been truncated (some time after this routine returns).
 */
bool afatfs_ftruncate(afatfsFilePtr_t file, afatfsFileCallback_t callback)
{
    afatfsTruncateFile_t *opState;

    if (afatfs_fileIsBusy(file))
        return false;

    file->operation.operation = AFATFS_FILE_OPERATION_TRUNCATE;

    opState = &file->operation.state.truncateFile;
    opState->callback = callback;
    opState->phase = AFATFS_TRUNCATE_FILE_INITIAL;
    opState->startCluster = file->firstCluster;
    opState->currentCluster = opState->startCluster;

#ifdef AFATFS_USE_FREEFILE
    if ((file->mode & AFATFS_FILE_MODE_CONTIGUOUS) != 0) {
        // The file is contiguous and ends where the freefile begins
        opState->endCluster = afatfs.freeFile.firstCluster;
    } else
#endif
    {
        // The range of clusters to delete is not contiguous, so follow it as a linked-list instead
        opState->endCluster = 0;
    }

    // We'll drop the cluster chain from the directory entry immediately
    file->firstCluster = 0;
    file->logicalSize = 0;
    file->physicalSize = 0;

    afatfs_fseek(file, 0, AFATFS_SEEK_SET);

    return true;
}

/**
 * Load details from the given FAT directory entry into the file.
 */
static void afatfs_fileLoadDirectoryEntry(afatfsFile_t *file, fatDirectoryEntry_t *entry)
{
    file->firstCluster = (uint32_t) (entry->firstClusterHigh << 16) | entry->firstClusterLow;
    file->logicalSize = entry->fileSize;
    file->physicalSize = roundUpTo(entry->fileSize, afatfs_clusterSize());
    file->attrib = entry->attrib;
}

static void afatfs_createFileContinue(afatfsFile_t *file)
{
    afatfsCreateFile_t *opState = &file->operation.state.createFile;
    fatDirectoryEntry_t *entry;
    afatfsOperationStatus_e status;

    doMore:

    switch (opState->phase) {
        case AFATFS_CREATEFILE_PHASE_INITIAL:
            afatfs_findFirst(&afatfs.currentDirectory, &file->directoryEntryPos);
            opState->phase = AFATFS_CREATEFILE_PHASE_FIND_FILE;
            goto doMore;
        break;
        case AFATFS_CREATEFILE_PHASE_FIND_FILE:
            do {
                status = afatfs_findNext(&afatfs.currentDirectory, &file->directoryEntryPos, &entry);

                switch (status) {
                    case AFATFS_OPERATION_SUCCESS:
                        // Is this the last entry in the directory?
                        if (entry == NULL || fat_isDirectoryEntryTerminator(entry)) {
                            afatfs_findLast(&afatfs.currentDirectory);

                            if ((file->mode & AFATFS_FILE_MODE_CREATE) != 0) {
                                // The file didn't already exist, so we can create it. Allocate a new directory entry
                                afatfs_findFirst(&afatfs.currentDirectory, &file->directoryEntryPos);

                                opState->phase = AFATFS_CREATEFILE_PHASE_CREATE_NEW_FILE;
                                goto doMore;
                            } else {
                                // File not found.

                                opState->phase = AFATFS_CREATEFILE_PHASE_FAILURE;
                                goto doMore;
                            }
                        } else if (entry->attrib & FAT_FILE_ATTRIBUTE_VOLUME_ID) {
                            break;
                        } else if (strncmp(entry->filename, (char*) opState->filename, FAT_FILENAME_LENGTH) == 0) {
                            // We found a file or directory with this name!

                            // Do not open file as dir or dir as file
                            if (((entry->attrib ^ file->attrib) & FAT_FILE_ATTRIBUTE_DIRECTORY) != 0) {
                                afatfs_findLast(&afatfs.currentDirectory);
                                opState->phase = AFATFS_CREATEFILE_PHASE_FAILURE;
                                goto doMore;
                            }

                            afatfs_fileLoadDirectoryEntry(file, entry);

                            afatfs_findLast(&afatfs.currentDirectory);

                            opState->phase = AFATFS_CREATEFILE_PHASE_SUCCESS;
                            goto doMore;
                        } // Else this entry doesn't match, fall through and continue the search
                    break;
                    case AFATFS_OPERATION_FAILURE:
                        afatfs_findLast(&afatfs.currentDirectory);
                        opState->phase = AFATFS_CREATEFILE_PHASE_FAILURE;
                        goto doMore;
                    break;
                    case AFATFS_OPERATION_IN_PROGRESS:
                        ;
                }
            } while (status == AFATFS_OPERATION_SUCCESS);
        break;
        case AFATFS_CREATEFILE_PHASE_CREATE_NEW_FILE:
            status = afatfs_allocateDirectoryEntry(&afatfs.currentDirectory, &entry, &file->directoryEntryPos);

            if (status == AFATFS_OPERATION_SUCCESS) {
                memset(entry, 0, sizeof(*entry));

                memcpy(entry->filename, opState->filename, FAT_FILENAME_LENGTH);
                entry->attrib = file->attrib;

                uint16_t fileDate = AFATFS_DEFAULT_FILE_DATE;
                uint16_t fileTime = AFATFS_DEFAULT_FILE_TIME;

                #ifdef USE_RTC_TIME
                // rtcGetDateTime will fill dt with 0000-01-01T00:00:00
                // when time is not known.
                dateTime_t dt, local_dt;
                rtcGetDateTime(&dt);
                if (dt.year != 0) {
                    // By tradition, FAT filesystem timestamps use local time.
                    dateTimeUTCToLocal(&dt, &local_dt);
                    fileDate = FAT_MAKE_DATE(local_dt.year, local_dt.month, local_dt.day);
                    fileTime = FAT_MAKE_TIME(local_dt.hours, local_dt.minutes, local_dt.seconds);
                }
                #endif

                entry->creationDate = fileDate;
                entry->creationTime = fileTime;
                entry->lastWriteDate = fileDate;
                entry->lastWriteTime = fileTime;

#ifdef AFATFS_DEBUG_VERBOSE
                fprintf(stderr, "Adding directory entry for %.*s to sector %u\n", FAT_FILENAME_LENGTH, opState->filename, file->directoryEntryPos.sectorNumberPhysical);
#endif

                opState->phase = AFATFS_CREATEFILE_PHASE_SUCCESS;
                goto doMore;
            } else if (status == AFATFS_OPERATION_FAILURE) {
                opState->phase = AFATFS_CREATEFILE_PHASE_FAILURE;
                goto doMore;
            }
        break;
        case AFATFS_CREATEFILE_PHASE_SUCCESS:
            if ((file->mode & AFATFS_FILE_MODE_RETAIN_DIRECTORY) != 0) {
                /*
                 * For this high performance file type, we require the directory entry for the file to be retained
                 * in the cache at all times.
                 */
                uint8_t *directorySector;

                status = afatfs_cacheSector(
                    file->directoryEntryPos.sectorNumberPhysical,
                    &directorySector,
                    AFATFS_CACHE_READ | AFATFS_CACHE_RETAIN,
                    0
                );

                if (status != AFATFS_OPERATION_SUCCESS) {
                    // Retry next time
                    break;
                }
            }

            afatfs_fseek(file, 0, AFATFS_SEEK_SET);

            // Is file empty?
            if (file->cursorCluster == 0) {
#ifdef AFATFS_USE_FREEFILE
                if ((file->mode & AFATFS_FILE_MODE_CONTIGUOUS) != 0) {
                    if (afatfs_fileIsBusy(&afatfs.freeFile)) {
                        // Someone else's using the freefile, come back later.
                        break;
                    } else {
                        // Lock the freefile for our exclusive access
                        afatfs.freeFile.operation.operation = AFATFS_FILE_OPERATION_LOCKED;
                    }
                }
#endif
            } else {
                // We can't guarantee that the existing file contents are contiguous
                file->mode &= ~AFATFS_FILE_MODE_CONTIGUOUS;

                // Seek to the end of the file if it is in append mode
                if ((file->mode & AFATFS_FILE_MODE_APPEND) != 0) {
                    // This replaces our open file operation
                    file->operation.operation = AFATFS_FILE_OPERATION_NONE;
                    afatfs_fseekInternal(file, file->logicalSize, opState->callback);
                    break;
                }

                // If we're only writing (not reading) the file must be truncated
                if (file->mode == (AFATFS_FILE_MODE_CREATE | AFATFS_FILE_MODE_WRITE)) {
                    // This replaces our open file operation
                    file->operation.operation = AFATFS_FILE_OPERATION_NONE;
                    afatfs_ftruncate(file, opState->callback);
                    break;
                }
            }

            file->operation.operation = AFATFS_FILE_OPERATION_NONE;
            opState->callback(file);
        break;
        case AFATFS_CREATEFILE_PHASE_FAILURE:
            file->type = AFATFS_FILE_TYPE_NONE;
            opState->callback(NULL);
        break;
    }
}

/**
 * Reset the in-memory data for the given handle back to the zeroed initial state
 */
static void afatfs_initFileHandle(afatfsFilePtr_t file)
{
    memset(file, 0, sizeof(*file));
    file->writeLockedCacheIndex = -1;
    file->readRetainCacheIndex = -1;
}

static void afatfs_funlinkContinue(afatfsFilePtr_t file)
{
    afatfsUnlinkFile_t *opState = &file->operation.state.unlinkFile;
    afatfsOperationStatus_e status;

    status = afatfs_ftruncateContinue(file, true);

    if (status == AFATFS_OPERATION_SUCCESS) {
        // Once the truncation is completed, we can close the file handle
        file->operation.operation = AFATFS_FILE_OPERATION_NONE;
        afatfs_fclose(file, opState->callback);
    }
}

/**
 * Delete and close the file.
 *
 * Returns true if the operation was successfully queued (callback will be called some time after this routine returns)
 * or false if the file is busy and you should try again later.
 */
bool afatfs_funlink(afatfsFilePtr_t file, afatfsCallback_t callback)
{
    afatfsUnlinkFile_t *opState = &file->operation.state.unlinkFile;

    if (!file || file->type == AFATFS_FILE_TYPE_NONE) {
        return true;
    }

    /*
     * Internally an unlink is implemented by first doing a ftruncate(), marking the directory entry as deleted,
     * then doing a fclose() operation.
     */

    // Start the sub-operation of truncating the file
    if (!afatfs_ftruncate(file, NULL))
        return false;

    /*
     * The unlink operation has its own private callback field so that the truncate suboperation doesn't end up
     * calling back early when it completes:
     */
    opState->callback = callback;

    file->operation.operation = AFATFS_FILE_OPERATION_UNLINK;

    return true;
}

/**
 * Open (or create) a file in the CWD with the given filename.
 *
 * file             - Memory location to store the newly opened file details
 * name             - Filename in "name.ext" format. No path.
 * attrib           - FAT file attributes to give the file (if created)
 * fileMode         - Bitset of AFATFS_FILE_MODE_* constants. Include AFATFS_FILE_MODE_CREATE to create the file if
 *                    it does not exist.
 * callback         - Called when the operation is complete
 */
static afatfsFilePtr_t afatfs_createFile(afatfsFilePtr_t file, const char *name, uint8_t attrib, uint8_t fileMode,
        afatfsFileCallback_t callback)
{
    afatfsCreateFile_t *opState = &file->operation.state.createFile;

    afatfs_initFileHandle(file);

    // Queue the operation to finish the file creation
    file->operation.operation = AFATFS_FILE_OPERATION_CREATE_FILE;

    file->mode = fileMode;

    if (strcmp(name, ".") == 0) {
        file->firstCluster = afatfs.currentDirectory.firstCluster;
        file->physicalSize = afatfs.currentDirectory.physicalSize;
        file->logicalSize = afatfs.currentDirectory.logicalSize;
        file->attrib = afatfs.currentDirectory.attrib;
        file->type = afatfs.currentDirectory.type;
    } else {
        fat_convertFilenameToFATStyle(name, opState->filename);
        file->attrib = attrib;

        if ((attrib & FAT_FILE_ATTRIBUTE_DIRECTORY) != 0) {
            file->type = AFATFS_FILE_TYPE_DIRECTORY;
        } else {
            file->type = AFATFS_FILE_TYPE_NORMAL;
        }
    }

    opState->callback = callback;

    if (strcmp(name, ".") == 0) {
        // Since we already have the directory entry details, we can skip straight to the final operations requried
        opState->phase = AFATFS_CREATEFILE_PHASE_SUCCESS;
    } else {
        opState->phase = AFATFS_CREATEFILE_PHASE_INITIAL;
    }

    afatfs_createFileContinue(file);

    return file;
}

static void afatfs_fcloseContinue(afatfsFilePtr_t file)
{
    afatfsCacheBlockDescriptor_t *descriptor;
    afatfsCloseFile_t *opState = &file->operation.state.closeFile;

    /*
     * Directories don't update their parent directory entries over time, because their fileSize field in the directory
     * never changes (when we add the first cluster to the directory we save the directory entry at that point and it
     * doesn't change afterwards). So don't bother trying to save their directory entries during fclose().
     *
     * Also if we only opened the file for read then we didn't change the directory entry either.
     */
    if (file->type != AFATFS_FILE_TYPE_DIRECTORY && file->type != AFATFS_FILE_TYPE_FAT16_ROOT_DIRECTORY
            && (file->mode & (AFATFS_FILE_MODE_APPEND | AFATFS_FILE_MODE_WRITE)) != 0) {
        if (afatfs_saveDirectoryEntry(file, AFATFS_SAVE_DIRECTORY_FOR_CLOSE) != AFATFS_OPERATION_SUCCESS) {
            return;
        }
    }

    // Release our reservation on the directory cache if needed
    if ((file->mode & AFATFS_FILE_MODE_RETAIN_DIRECTORY) != 0) {
        descriptor = afatfs_findCacheSector(file->directoryEntryPos.sectorNumberPhysical);

        if (descriptor) {
            descriptor->retainCount = MAX((int) descriptor->retainCount - 1, 0);
        }
    }

    // Release locks on the sector at the file cursor position
    afatfs_fileUnlockCacheSector(file);

#ifdef AFATFS_USE_FREEFILE
    // Release our exclusive lock on the freefile if needed
    if ((file->mode & AFATFS_FILE_MODE_CONTIGUOUS) != 0) {
        afatfs_assert(afatfs.freeFile.operation.operation == AFATFS_FILE_OPERATION_LOCKED);
        afatfs.freeFile.operation.operation = AFATFS_FILE_OPERATION_NONE;
    }
#endif

    file->type = AFATFS_FILE_TYPE_NONE;
    file->operation.operation = AFATFS_FILE_OPERATION_NONE;

    if (opState->callback) {
        opState->callback();
    }
}

/**
 * Returns true if an operation was successfully queued to close the file and destroy the file handle. If the file is
 * currently busy, false is returned and you should retry later.
 *
 * If provided, the callback will be called after the operation completes (pass NULL for no callback).
 *
 * If this function returns true, you should not make any further calls to the file (as the handle might be reused for a
 * new file).
 */
bool afatfs_fclose(afatfsFilePtr_t file, afatfsCallback_t callback)
{
    if (!file || file->type == AFATFS_FILE_TYPE_NONE) {
        return true;
    } else if (afatfs_fileIsBusy(file)) {
        return false;
    } else {
        afatfs_fileUpdateFilesize(file);

        file->operation.operation = AFATFS_FILE_OPERATION_CLOSE;
        file->operation.state.closeFile.callback = callback;
        afatfs_fcloseContinue(file);
        return true;
    }
}

/**
 * Create a new directory with the given name, or open the directory if it already exists.
 *
 * The directory will be passed to the callback, or NULL if the creation failed.
 *
 * Returns true if the directory creation was begun, or false if there are too many open files.
 */
bool afatfs_mkdir(const char *filename, afatfsFileCallback_t callback)
{
    afatfsFilePtr_t file = afatfs_allocateFileHandle();

    if (file) {
        afatfs_createFile(file, filename, FAT_FILE_ATTRIBUTE_DIRECTORY, AFATFS_FILE_MODE_CREATE | AFATFS_FILE_MODE_READ | AFATFS_FILE_MODE_WRITE, callback);
    } else if (callback) {
        callback(NULL);
    }

    return file != NULL;
}

/**
 * Change the working directory to the directory with the given handle (use fopen). Pass NULL for `directory` in order to
 * change to the root directory.
 *
 * Returns true on success, false if you should call again later to retry. After changing into a directory, your handle
 * to that directory may be closed by fclose().
 */
bool afatfs_chdir(afatfsFilePtr_t directory)
{
    if (afatfs_fileIsBusy(&afatfs.currentDirectory)) {
        return false;
    }

    if (directory) {
        if (afatfs_fileIsBusy(directory)) {
            return false;
        }

        memcpy(&afatfs.currentDirectory, directory, sizeof(*directory));
        return true;
    } else {
        afatfs_initFileHandle(&afatfs.currentDirectory);

        afatfs.currentDirectory.mode = AFATFS_FILE_MODE_READ | AFATFS_FILE_MODE_WRITE;

        if (afatfs.filesystemType == FAT_FILESYSTEM_TYPE_FAT16)
            afatfs.currentDirectory.type = AFATFS_FILE_TYPE_FAT16_ROOT_DIRECTORY;
        else
            afatfs.currentDirectory.type = AFATFS_FILE_TYPE_DIRECTORY;

        afatfs.currentDirectory.firstCluster = afatfs.rootDirectoryCluster;
        afatfs.currentDirectory.attrib = FAT_FILE_ATTRIBUTE_DIRECTORY;

        // Root directories don't have a directory entry to represent themselves:
        afatfs.currentDirectory.directoryEntryPos.sectorNumberPhysical = 0;

        afatfs_fseek(&afatfs.currentDirectory, 0, AFATFS_SEEK_SET);

        return true;
    }
}

/**
 * Begin the process of opening a file with the given name in the current working directory (paths in the filename are
 * not supported) using the given mode.
 *
 * To open the current working directory, pass "." for filename.
 *
 * The complete() callback is called when finished with either a file handle (file was opened) or NULL upon failure.
 *
 * Supported file mode strings:
 *
 * r - Read from an existing file
 * w - Create a file for write access, if the file already exists then truncate it
 * a - Create a file for write access to the end of the file only, if the file already exists then append to it
 *
 * r+ - Read and write from an existing file
 * w+ - Read and write from an existing file, if the file doesn't already exist it is created
 * a+ - Read from or append to an existing file, if the file doesn't already exist it is created TODO
 *
 * as - Create a new file which is stored contiguously on disk (high performance mode/freefile) for append or write
 * ws   If the file is already non-empty or freefile support is not compiled in then it will fall back to non-contiguous
 *      operation.
 *
 * All other mode strings are illegal. In particular, don't add "b" to the end of the mode string.
 *
 * Returns false if the the open failed really early (out of file handles).
 */
bool afatfs_fopen(const char *filename, const char *mode, afatfsFileCallback_t complete)
{
    uint8_t fileMode = 0;
    afatfsFilePtr_t file;

    switch (mode[0]) {
        case 'r':
            fileMode = AFATFS_FILE_MODE_READ;
        break;
        case 'w':
            fileMode = AFATFS_FILE_MODE_WRITE | AFATFS_FILE_MODE_CREATE;
        break;
        case 'a':
            fileMode = AFATFS_FILE_MODE_APPEND | AFATFS_FILE_MODE_CREATE;
        break;
    }

    switch (mode[1]) {
        case '+':
            fileMode |= AFATFS_FILE_MODE_READ;

            if (fileMode == AFATFS_FILE_MODE_READ) {
                fileMode |= AFATFS_FILE_MODE_WRITE;
            }
        break;
        case 's':
#ifdef AFATFS_USE_FREEFILE
            fileMode |= AFATFS_FILE_MODE_CONTIGUOUS | AFATFS_FILE_MODE_RETAIN_DIRECTORY;
#endif
        break;
    }

    file = afatfs_allocateFileHandle();

    if (file) {
        afatfs_createFile(file, filename, FAT_FILE_ATTRIBUTE_ARCHIVE, fileMode, complete);
    } else if (complete) {
        complete(NULL);
    }

    return file != NULL;
}

/**
 * Write a single character to the file at the current cursor position. If the cache is too busy to accept the write,
 * it is silently dropped.
 */
void afatfs_fputc(afatfsFilePtr_t file, uint8_t c)
{
    uint32_t cursorOffsetInSector = file->cursorOffset % AFATFS_SECTOR_SIZE;

    int cacheIndex = file->writeLockedCacheIndex;

    /* If we've already locked the current sector in the cache, and we won't be completing the sector, we won't
     * be caching/uncaching/seeking, so we can just run this simpler fast case.
     */
    if (cacheIndex != -1 && cursorOffsetInSector != AFATFS_SECTOR_SIZE - 1) {
        afatfs_cacheSectorGetMemory(cacheIndex)[cursorOffsetInSector] = c;
        file->cursorOffset++;
    } else {
        // Slow path
        afatfs_fwrite(file, &c, sizeof(c));
    }
}

/**
 * Attempt to write `len` bytes from `buffer` into the `file`.
 *
 * Returns the number of bytes actually written.
 *
 * 0 will be returned when:
 *     The filesystem is busy (try again later)
 *
 * Fewer bytes will be written than requested when:
 *     The write spanned a sector boundary and the next sector's contents/location was not yet available in the cache.
 *     Or you tried to extend the length of the file but the filesystem is full (check afatfs_isFull()).
 */
uint32_t afatfs_fwrite(afatfsFilePtr_t file, const uint8_t *buffer, uint32_t len)
{
    if ((file->mode & (AFATFS_FILE_MODE_APPEND | AFATFS_FILE_MODE_WRITE)) == 0) {
        return 0;
    }

    if (afatfs_fileIsBusy(file)) {
        // There might be a seek pending
        return 0;
    }

    uint32_t cursorOffsetInSector = file->cursorOffset % AFATFS_SECTOR_SIZE;
    uint32_t writtenBytes = 0;

    while (len > 0) {
        uint32_t bytesToWriteThisSector = MIN(AFATFS_SECTOR_SIZE - cursorOffsetInSector, len);
        uint8_t *sectorBuffer;

        sectorBuffer = afatfs_fileLockCursorSectorForWrite(file);
        if (!sectorBuffer) {
            // Cache is currently busy
            break;
        }

        memcpy(sectorBuffer + cursorOffsetInSector, buffer, bytesToWriteThisSector);

        writtenBytes += bytesToWriteThisSector;

        /*
         * If the seek doesn't complete immediately then we'll break and wait for that seek to complete by waiting for
         * the file to be non-busy on entry again.
         *
         * A seek operation should always be able to queue on the file since we have checked that the file wasn't busy
         * on entry (fseek will never return AFATFS_OPERATION_FAILURE).
         *
         * If the seek has to queue, when the seek completes, it'll update the fileSize for us to contain the cursor.
         */
        if (afatfs_fseekInternal(file, bytesToWriteThisSector, NULL) == AFATFS_OPERATION_IN_PROGRESS) {
            break;
        }

#ifdef AFATFS_USE_FREEFILE
        if ((file->mode & AFATFS_FILE_MODE_CONTIGUOUS) != 0) {
            afatfs_assert(file->cursorCluster < afatfs.freeFile.firstCluster);
        }
#endif

        len -= bytesToWriteThisSector;
        buffer += bytesToWriteThisSector;
        cursorOffsetInSector = 0;
    }

    return writtenBytes;
}

/**
 * Attempt to read `len` bytes from `file` into the `buffer`.
 *
 * Returns the number of bytes actually read.
 *
 * 0 will be returned when:
 *     The filesystem is busy (try again later)
 *     EOF was reached (check afatfs_isEof())
 *
 * Fewer bytes than requested will be read when:
 *     The read spans a AFATFS_SECTOR_SIZE boundary and the following sector was not available in the cache yet.
 */
uint32_t afatfs_fread(afatfsFilePtr_t file, uint8_t *buffer, uint32_t len)
{
    if ((file->mode & AFATFS_FILE_MODE_READ) == 0) {
        return 0;
    }

    if (afatfs_fileIsBusy(file)) {
        // There might be a seek pending
        return 0;
    }

    /*
     * If we've just previously fwritten() to extend the file, the logical filesize will be out of date and the cursor
     * will appear to be beyond the end of the file (but actually it's precisely at the end of the file, because if
     * we had seeked backwards to where we could read something with fseek(), we would have updated the filesize).
     */
    if (file->cursorOffset >= file->logicalSize)
        return 0;

    len = MIN(file->logicalSize - file->cursorOffset, len);

    uint32_t readBytes = 0;
    uint32_t cursorOffsetInSector = file->cursorOffset % AFATFS_SECTOR_SIZE;

    while (len > 0) {
        uint32_t bytesToReadThisSector = MIN(AFATFS_SECTOR_SIZE - cursorOffsetInSector, len);
        uint8_t *sectorBuffer;

        sectorBuffer = afatfs_fileRetainCursorSectorForRead(file);
        if (!sectorBuffer) {
            // Cache is currently busy
            return readBytes;
        }

        memcpy(buffer, sectorBuffer + cursorOffsetInSector, bytesToReadThisSector);

        readBytes += bytesToReadThisSector;

        /*
         * If the seek doesn't complete immediately then we'll break and wait for that seek to complete by waiting for
         * the file to be non-busy on entry again.
         *
         * A seek operation should always be able to queue on the file since we have checked that the file wasn't busy
         * on entry (fseek will never return AFATFS_OPERATION_FAILURE).
         */
        if (afatfs_fseekInternal(file, bytesToReadThisSector, NULL) == AFATFS_OPERATION_IN_PROGRESS) {
            break;
        }

        len -= bytesToReadThisSector;
        buffer += bytesToReadThisSector;
        cursorOffsetInSector = 0;
    }

    return readBytes;
}

/**
 * Returns true if the file's pointer position currently lies at the end-of-file point (i.e. one byte beyond the last
 * byte in the file).
 */
bool afatfs_feof(afatfsFilePtr_t file)
{
    return file->cursorOffset >= file->logicalSize;
}

/**
 * Continue any queued operations on the given file.
 */
static void afatfs_fileOperationContinue(afatfsFile_t *file)
{
    if (file->type == AFATFS_FILE_TYPE_NONE)
        return;

    switch (file->operation.operation) {
        case AFATFS_FILE_OPERATION_CREATE_FILE:
            afatfs_createFileContinue(file);
        break;
        case AFATFS_FILE_OPERATION_SEEK:
            afatfs_fseekInternalContinue(file);
        break;
        case AFATFS_FILE_OPERATION_CLOSE:
            afatfs_fcloseContinue(file);
        break;
        case AFATFS_FILE_OPERATION_UNLINK:
             afatfs_funlinkContinue(file);
        break;
        case AFATFS_FILE_OPERATION_TRUNCATE:
            afatfs_ftruncateContinue(file, false);
        break;
#ifdef AFATFS_USE_FREEFILE
        case AFATFS_FILE_OPERATION_APPEND_SUPERCLUSTER:
            afatfs_appendSuperclusterContinue(file);
        break;
        case AFATFS_FILE_OPERATION_LOCKED:
            ;
        break;
#endif
        case AFATFS_FILE_OPERATION_APPEND_FREE_CLUSTER:
            afatfs_appendRegularFreeClusterContinue(file);
        break;
        case AFATFS_FILE_OPERATION_EXTEND_SUBDIRECTORY:
            afatfs_extendSubdirectoryContinue(file);
        break;
        case AFATFS_FILE_OPERATION_NONE:
            ;
        break;
    }
}

/**
 * Check files for pending operations and execute them.
 */
static void afatfs_fileOperationsPoll(void)
{
    afatfs_fileOperationContinue(&afatfs.currentDirectory);

#ifdef AFATFS_USE_INTROSPECTIVE_LOGGING
    afatfs_fileOperationContinue(&afatfs.introSpecLog);
#endif

    for (int i = 0; i < AFATFS_MAX_OPEN_FILES; i++) {
        afatfs_fileOperationContinue(&afatfs.openFiles[i]);
    }
}

#ifdef AFATFS_USE_FREEFILE

/**
 * Return the available size of the freefile (used for files in contiguous append mode)
 */
uint32_t afatfs_getContiguousFreeSpace(void)
{
    return afatfs.freeFile.logicalSize;
}

/**
 * Call to set up the initial state for finding the largest block of free space on the device whose corresponding FAT
 * sectors are themselves entirely free space (so the free space has dedicated FAT sectors of its own).
 */
static void afatfs_findLargestContiguousFreeBlockBegin(void)
{
    // The first FAT sector has two reserved entries, so it isn't eligible for this search. Start at the next FAT sector.
    afatfs.initState.freeSpaceSearch.candidateStart = afatfs_fatEntriesPerSector();
    afatfs.initState.freeSpaceSearch.candidateEnd = afatfs.initState.freeSpaceSearch.candidateStart;
    afatfs.initState.freeSpaceSearch.bestGapStart = 0;
    afatfs.initState.freeSpaceSearch.bestGapLength = 0;
    afatfs.initState.freeSpaceSearch.phase = AFATFS_FREE_SPACE_SEARCH_PHASE_FIND_HOLE;
}

/**
 * Call to continue the search for the largest contiguous block of free space on the device.
 *
 * Returns:
 *     AFATFS_OPERATION_IN_PROGRESS - SD card is busy, call again later to resume
 *     AFATFS_OPERATION_SUCCESS - When the search has finished and afatfs.initState.freeSpaceSearch has been updated with the details of the best gap.
 *     AFATFS_OPERATION_FAILURE - When a read error occured
 */
static afatfsOperationStatus_e afatfs_findLargestContiguousFreeBlockContinue(void)
{
    afatfsFreeSpaceSearch_t *opState = &afatfs.initState.freeSpaceSearch;
    uint32_t fatEntriesPerSector = afatfs_fatEntriesPerSector();
    uint32_t candidateGapLength, searchLimit;
    afatfsFindClusterStatus_e searchStatus;

    while (1) {
        switch (opState->phase) {
            case AFATFS_FREE_SPACE_SEARCH_PHASE_FIND_HOLE:
                // Find the first free cluster
                switch (afatfs_findClusterWithCondition(CLUSTER_SEARCH_FREE_AT_BEGINNING_OF_FAT_SECTOR, &opState->candidateStart, afatfs.numClusters + FAT_SMALLEST_LEGAL_CLUSTER_NUMBER)) {
                    case AFATFS_FIND_CLUSTER_FOUND:
                        opState->candidateEnd = opState->candidateStart + 1;
                        opState->phase = AFATFS_FREE_SPACE_SEARCH_PHASE_GROW_HOLE;
                    break;

                    case AFATFS_FIND_CLUSTER_FATAL:
                        // Some sort of read error occured
                        return AFATFS_OPERATION_FAILURE;

                    case AFATFS_FIND_CLUSTER_NOT_FOUND:
                        // We finished searching the volume (didn't find any more holes to examine)
                        return AFATFS_OPERATION_SUCCESS;

                    case AFATFS_FIND_CLUSTER_IN_PROGRESS:
                        return AFATFS_OPERATION_IN_PROGRESS;
                }
            break;
            case AFATFS_FREE_SPACE_SEARCH_PHASE_GROW_HOLE:
                // Find the first used cluster after the beginning of the hole (that signals the end of the hole)

                // Don't search beyond the end of the volume, or such that the freefile size would exceed the max filesize
                searchLimit = MIN((uint64_t) opState->candidateStart + FAT_MAXIMUM_FILESIZE / afatfs_clusterSize(), afatfs.numClusters + FAT_SMALLEST_LEGAL_CLUSTER_NUMBER);

                searchStatus = afatfs_findClusterWithCondition(CLUSTER_SEARCH_OCCUPIED, &opState->candidateEnd, searchLimit);

                switch (searchStatus) {
                    case AFATFS_FIND_CLUSTER_NOT_FOUND:
                    case AFATFS_FIND_CLUSTER_FOUND:
                        // Either we found a used sector, or the search reached the end of the volume or exceeded the max filesize
                        candidateGapLength = opState->candidateEnd - opState->candidateStart;

                        if (candidateGapLength > opState->bestGapLength) {
                            opState->bestGapStart = opState->candidateStart;
                            opState->bestGapLength = candidateGapLength;
                        }

                        if (searchStatus == AFATFS_FIND_CLUSTER_NOT_FOUND) {
                            // This is the best hole there can be
                            return AFATFS_OPERATION_SUCCESS;
                        } else {
                            // Start a new search for a new hole
                            opState->candidateStart = roundUpTo(opState->candidateEnd + 1, fatEntriesPerSector);
                            opState->phase = AFATFS_FREE_SPACE_SEARCH_PHASE_FIND_HOLE;
                        }
                    break;

                    case AFATFS_FIND_CLUSTER_FATAL:
                        // Some sort of read error occured
                        return AFATFS_OPERATION_FAILURE;

                    case AFATFS_FIND_CLUSTER_IN_PROGRESS:
                        return AFATFS_OPERATION_IN_PROGRESS;
                }
            break;
        }
    }
}

static void afatfs_freeFileCreated(afatfsFile_t *file)
{
    if (file) {
        // Did the freefile already have allocated space?
        if (file->logicalSize > 0) {
            // We've completed freefile init, move on to the next init phase
            afatfs.initPhase = AFATFS_INITIALIZATION_FREEFILE_LAST + 1;
        } else {
            // Allocate clusters for the freefile
            afatfs_findLargestContiguousFreeBlockBegin();
            afatfs.initPhase = AFATFS_INITIALIZATION_FREEFILE_FAT_SEARCH;
        }
    } else {
        // Failed to allocate an entry
        afatfs.lastError = AFATFS_ERROR_GENERIC;
        afatfs.filesystemState = AFATFS_FILESYSTEM_STATE_FATAL;
    }
}

#endif

#ifdef AFATFS_USE_INTROSPECTIVE_LOGGING

static void afatfs_introspecLogCreated(afatfsFile_t *file)
{
    if (file) {
        afatfs.initPhase++;
    } else {
        afatfs.lastError = AFATFS_ERROR_GENERIC;
        afatfs.filesystemState = AFATFS_FILESYSTEM_STATE_FATAL;
    }
}

#endif

static void afatfs_initContinue(void)
{
#ifdef AFATFS_USE_FREEFILE
    afatfsOperationStatus_e status;
#endif

    uint8_t *sector;

    doMore:

    switch (afatfs.initPhase) {
        case AFATFS_INITIALIZATION_READ_MBR:
            if (afatfs_cacheSector(0, &sector, AFATFS_CACHE_READ | AFATFS_CACHE_DISCARDABLE, 0) == AFATFS_OPERATION_SUCCESS) {
                if (afatfs_parseMBR(sector)) {
                    afatfs.initPhase = AFATFS_INITIALIZATION_READ_VOLUME_ID;
                    goto doMore;
                } else {
                    afatfs.lastError = AFATFS_ERROR_BAD_MBR;
                    afatfs.filesystemState = AFATFS_FILESYSTEM_STATE_FATAL;
                }
            }
        break;
        case AFATFS_INITIALIZATION_READ_VOLUME_ID:
            if (afatfs_cacheSector(afatfs.partitionStartSector, &sector, AFATFS_CACHE_READ | AFATFS_CACHE_DISCARDABLE, 0) == AFATFS_OPERATION_SUCCESS) {
                if (afatfs_parseVolumeID(sector)) {
                    // Open the root directory
                    afatfs_chdir(NULL);

                    afatfs.initPhase++;
                } else {
                    afatfs.lastError = AFATFS_ERROR_BAD_FILESYSTEM_HEADER;
                    afatfs.filesystemState = AFATFS_FILESYSTEM_STATE_FATAL;
                }
            }
        break;

#ifdef AFATFS_USE_FREEFILE
        case AFATFS_INITIALIZATION_FREEFILE_CREATE:
            afatfs.initPhase = AFATFS_INITIALIZATION_FREEFILE_CREATING;

            afatfs_createFile(&afatfs.freeFile, AFATFS_FREESPACE_FILENAME, FAT_FILE_ATTRIBUTE_SYSTEM | FAT_FILE_ATTRIBUTE_READ_ONLY,
                AFATFS_FILE_MODE_CREATE | AFATFS_FILE_MODE_RETAIN_DIRECTORY, afatfs_freeFileCreated);
        break;
        case AFATFS_INITIALIZATION_FREEFILE_CREATING:
            afatfs_fileOperationContinue(&afatfs.freeFile);
        break;
        case AFATFS_INITIALIZATION_FREEFILE_FAT_SEARCH:
            if (afatfs_findLargestContiguousFreeBlockContinue() == AFATFS_OPERATION_SUCCESS) {
                // If the freefile ends up being empty then we only have to save its directory entry:
                afatfs.initPhase = AFATFS_INITIALIZATION_FREEFILE_SAVE_DIR_ENTRY;

                if (afatfs.initState.freeSpaceSearch.bestGapLength > AFATFS_FREEFILE_LEAVE_CLUSTERS + 1) {
                    afatfs.initState.freeSpaceSearch.bestGapLength -= AFATFS_FREEFILE_LEAVE_CLUSTERS;

                    /* So that the freefile never becomes empty, we want it to occupy a non-integer number of
                     * superclusters. So its size mod the number of clusters in a supercluster should be 1.
                     */
                    afatfs.initState.freeSpaceSearch.bestGapLength = ((afatfs.initState.freeSpaceSearch.bestGapLength - 1) & ~(afatfs_fatEntriesPerSector() - 1)) + 1;

                    // Anything useful left over?
                    if (afatfs.initState.freeSpaceSearch.bestGapLength > afatfs_fatEntriesPerSector()) {
                        uint32_t startCluster = afatfs.initState.freeSpaceSearch.bestGapStart;
                        // Points 1-beyond the final cluster of the freefile:
                        uint32_t endCluster = afatfs.initState.freeSpaceSearch.bestGapStart + afatfs.initState.freeSpaceSearch.bestGapLength;

                        afatfs_assert(endCluster < afatfs.numClusters + FAT_SMALLEST_LEGAL_CLUSTER_NUMBER);

                        afatfs.initState.freeSpaceFAT.startCluster = startCluster;
                        afatfs.initState.freeSpaceFAT.endCluster = endCluster;

                        afatfs.freeFile.firstCluster = startCluster;

                        afatfs.freeFile.logicalSize = afatfs.initState.freeSpaceSearch.bestGapLength * afatfs_clusterSize();
                        afatfs.freeFile.physicalSize = afatfs.freeFile.logicalSize;

                        // We can write the FAT table for the freefile now
                        afatfs.initPhase = AFATFS_INITIALIZATION_FREEFILE_UPDATE_FAT;
                    } // Else the freefile's FAT chain and filesize remains the default (empty)
                }

                goto doMore;
            }
        break;
        case AFATFS_INITIALIZATION_FREEFILE_UPDATE_FAT:
            status = afatfs_FATFillWithPattern(AFATFS_FAT_PATTERN_TERMINATED_CHAIN, &afatfs.initState.freeSpaceFAT.startCluster, afatfs.initState.freeSpaceFAT.endCluster);

            if (status == AFATFS_OPERATION_SUCCESS) {
                afatfs.initPhase = AFATFS_INITIALIZATION_FREEFILE_SAVE_DIR_ENTRY;

                goto doMore;
            } else if (status == AFATFS_OPERATION_FAILURE) {
                afatfs.lastError = AFATFS_ERROR_GENERIC;
                afatfs.filesystemState = AFATFS_FILESYSTEM_STATE_FATAL;
            }
        break;
        case AFATFS_INITIALIZATION_FREEFILE_SAVE_DIR_ENTRY:
            status = afatfs_saveDirectoryEntry(&afatfs.freeFile, AFATFS_SAVE_DIRECTORY_NORMAL);

            if (status == AFATFS_OPERATION_SUCCESS) {
                afatfs.initPhase++;
                goto doMore;
            } else if (status == AFATFS_OPERATION_FAILURE) {
                afatfs.lastError = AFATFS_ERROR_GENERIC;
                afatfs.filesystemState = AFATFS_FILESYSTEM_STATE_FATAL;
            }
        break;
#endif

#ifdef AFATFS_USE_INTROSPECTIVE_LOGGING
        case AFATFS_INITIALIZATION_INTROSPEC_LOG_CREATE:
            afatfs.initPhase = AFATFS_INITIALIZATION_INTROSPEC_LOG_CREATING;

            afatfs_createFile(&afatfs.introSpecLog, AFATFS_INTROSPEC_LOG_FILENAME, FAT_FILE_ATTRIBUTE_ARCHIVE,
                AFATFS_FILE_MODE_CREATE | AFATFS_FILE_MODE_APPEND, afatfs_introspecLogCreated);
        break;
        case AFATFS_INITIALIZATION_INTROSPEC_LOG_CREATING:
            afatfs_fileOperationContinue(&afatfs.introSpecLog);
        break;
#endif

        case AFATFS_INITIALIZATION_DONE:
            afatfs.filesystemState = AFATFS_FILESYSTEM_STATE_READY;
        break;
    }
}

/**
 * Check to see if there are any pending operations on the filesystem and perform a little work (without waiting on the
 * sdcard). You must call this periodically.
 */
void afatfs_poll(void)
{
    // Only attempt to continue FS operations if the card is present & ready, otherwise we would just be wasting time
    if (sdcard_poll()) {
        afatfs_flush();

        switch (afatfs.filesystemState) {
            case AFATFS_FILESYSTEM_STATE_INITIALIZATION:
                afatfs_initContinue();
            break;
            case AFATFS_FILESYSTEM_STATE_READY:
                afatfs_fileOperationsPoll();
            break;
            default:
                ;
        }
    }
}

#ifdef AFATFS_USE_INTROSPECTIVE_LOGGING

void afatfs_sdcardProfilerCallback(sdcardBlockOperation_e operation, uint32_t blockIndex, uint32_t duration)
{
    // Make sure the log file has actually been opened before we try to log to it:
    if (afatfs.introSpecLog.type == AFATFS_FILE_TYPE_NONE) {
        return;
    }

    enum {
        LOG_ENTRY_SIZE = 16 // Log entry size should be a power of two to avoid partial fwrites()
    };

    uint8_t buffer[LOG_ENTRY_SIZE];

    buffer[0] = operation;

    // Padding/reserved:
    buffer[1] = 0;
    buffer[2] = 0;
    buffer[3] = 0;

    buffer[4] = blockIndex & 0xFF;
    buffer[5] = (blockIndex >> 8) & 0xFF;
    buffer[6] = (blockIndex >> 16) & 0xFF;
    buffer[7] = (blockIndex >> 24) & 0xFF;

    buffer[8] = duration & 0xFF;
    buffer[9] = (duration >> 8) & 0xFF;
    buffer[10] = (duration >> 16) & 0xFF;
    buffer[11] = (duration >> 24) & 0xFF;

    // Padding/reserved:
    buffer[12] = 0;
    buffer[13] = 0;
    buffer[14] = 0;
    buffer[15] = 0;

    // Ignore write failures
    afatfs_fwrite(&afatfs.introSpecLog, buffer, LOG_ENTRY_SIZE);
}

#endif

afatfsFilesystemState_e afatfs_getFilesystemState(void)
{
    return afatfs.filesystemState;
}

afatfsError_e afatfs_getLastError(void)
{
    return afatfs.lastError;
}

void afatfs_init(void)
{
#ifdef STM32H7
    afatfs.cache = afatfs_cache;
#endif
    afatfs.filesystemState = AFATFS_FILESYSTEM_STATE_INITIALIZATION;
    afatfs.initPhase = AFATFS_INITIALIZATION_READ_MBR;
    afatfs.lastClusterAllocated = FAT_SMALLEST_LEGAL_CLUSTER_NUMBER;

#ifdef AFATFS_USE_INTROSPECTIVE_LOGGING
    sdcard_setProfilerCallback(afatfs_sdcardProfilerCallback);
#endif
}

/**
 * Shut down the filesystem, flushing all data to the disk. Keep calling until it returns true.
 *
 * dirty - Set to true to skip the flush operation and terminate immediately (buffered data will be lost!)
 */
bool afatfs_destroy(bool dirty)
{
    // Only attempt detailed cleanup if the filesystem is in reasonable looking state
    if (!dirty && afatfs.filesystemState == AFATFS_FILESYSTEM_STATE_READY) {
        int openFileCount = 0;

        for (int i = 0; i < AFATFS_MAX_OPEN_FILES; i++) {
            if (afatfs.openFiles[i].type != AFATFS_FILE_TYPE_NONE) {
                afatfs_fclose(&afatfs.openFiles[i], NULL);
                // The close operation might not finish right away, so count this file as still open for now
                openFileCount++;
            }
        }

#ifdef AFATFS_USE_INTROSPECTIVE_LOGGING
        if (afatfs.introSpecLog.type != AFATFS_FILE_TYPE_NONE) {
            afatfs_fclose(&afatfs.introSpecLog, NULL);
            openFileCount++;
        }
#endif

#ifdef AFATFS_USE_FREEFILE
        if (afatfs.freeFile.type != AFATFS_FILE_TYPE_NONE) {
            afatfs_fclose(&afatfs.freeFile, NULL);
            openFileCount++;
        }
#endif

        if (afatfs.currentDirectory.type != AFATFS_FILE_TYPE_NONE) {
            afatfs_fclose(&afatfs.currentDirectory, NULL);
            openFileCount++;
        }

        afatfs_poll();

        if (!afatfs_flush()) {
            return false;
        }

        if (afatfs.cacheFlushInProgress) {
            return false;
        }

        if (openFileCount > 0) {
            return false;
        }

#ifdef AFATFS_DEBUG
        /* All sector locks should have been released by closing the files, so the subsequent flush should have written
         * all dirty pages to disk. If not, something's wrong:
         */
        for (int i = 0; i < AFATFS_NUM_CACHE_SECTORS; i++) {
            afatfs_assert(afatfs.cacheDescriptor[i].state != AFATFS_CACHE_STATE_DIRTY);
        }
#endif
    }

    // Clear the afatfs so it's as if we never ran
    memset(&afatfs, 0, sizeof(afatfs));

    return true;
}

/**
 * Get a pessimistic estimate of the amount of buffer space that we have available to write to immediately.
 */
uint32_t afatfs_getFreeBufferSpace(void)
{
    uint32_t result = 0;
    for (int i = 0; i < AFATFS_NUM_CACHE_SECTORS; i++) {
        if (!afatfs.cacheDescriptor[i].locked && (afatfs.cacheDescriptor[i].state == AFATFS_CACHE_STATE_EMPTY || afatfs.cacheDescriptor[i].state == AFATFS_CACHE_STATE_IN_SYNC)) {
            result += AFATFS_SECTOR_SIZE;
        }
    }
    return result;
}
