/* ============================================================================
Copyright (c) 2007-2014, Broadcom Corporation
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
3. Neither the name of the copyright holder nor the names of its contributors
   may be used to endorse or promote products derived from this software without
   specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
============================================================================ */

#include "ghw_allocator_impl.h"

#include "ghw_memblock.h"

extern "C" {


#include "v3d2.h"
#include <sys/ioctl.h>
#include <fcntl.h>
#include <sys/mman.h>

#define DEVICE_NAME "/dev/v3d2"


};
namespace ghw {

GhwMemAllocator* GhwMemAllocator::create(u32 mode , u32 slab_size , u32 alignment )
{
    GhwAllocatorImpl* allocator = new GhwAllocatorImpl(mode,slab_size,alignment);
    if(allocator->initCheck()) {
		LOGE("allocator initcheck failed");
        delete allocator;
        return NULL;
    }

    return allocator;
}

int GhwAllocatorDevice::count = 0;
int GhwAllocatorImpl::count = 0;

GhwAllocatorDevice::GhwAllocatorDevice() :mFd(-1) {
	mFd = open(DEVICE_NAME, O_RDWR);
	if(mFd <= 0 ) {
		LOGE("GhwAllocatorDevice device open failed for %s\n",DEVICE_NAME);
	}
	count++;
	pthread_mutex_init(&mLock,NULL);
}

GhwAllocatorDevice::~GhwAllocatorDevice() {
	pthread_mutex_destroy(&mLock);
	count--;
}

void GhwAllocatorDevice::setMode(u32 mode) {
	mMode = mode;
}

int GhwAllocatorDevice::initCheck() {
	if(mFd > 0) return 0;
	else return -1;
}

void* GhwAllocatorDevice::allocDevMem(u32& pa, unsigned char*& va,u32 size) {

	int tempSize = size;
	int ret;
	int pgsize = 4096;//getpagesize();
	tempSize = (tempSize + pgsize-1) & (~(pgsize - 1));
	struct mem_alloc_request params;
	params.size = tempSize;

	/* get memory linear memory buffers */
	
	if(ioctl(mFd, V3D2_MEM_ALLOC, &params))
	{
		LOGE("Alloc failed for size %d\n",tempSize);
		return 0;
	}
	
	struct V3D2MemoryReference *ref = new struct V3D2MemoryReference;
	ref->handle = params.handle;
	ref->phys = params.physicalAddress;
	ref->size = params.size;
	ref->virt = 0;
	ref->mapcount = 0;
	
	ret = ioctl(v3d2_get_fd(),V3D2_MEM_SELECT,&ref->handle);
	assert(ret == 0);
	ref->virt = mmap(0,ref->size,PROT_READ|PROT_WRITE,MAP_SHARED,mFd,0);
	if ((int)ref->virt == -1) {
		printf("mmap error: %s\n",strerror(errno));
		return NULL;
	}

    return (void *)ref->virt;
}

void GhwAllocatorDevice::freeDevMem(u32& pa, unsigned char*& va,u32 size, void* handle) {

	int tempSize = size;
	int pgsize = 4096;//getpagesize();
	tempSize = (tempSize + pgsize-1) & (~(pgsize - 1));

    munmap(va, tempSize);
    ioctl(mFd, V3D2_MEM_FREE, handle);
	va = 0;
	pa = 0;

};

void GhwAllocatorDevice::protect() {
	pthread_mutex_lock(&mLock);
}

void GhwAllocatorDevice::unprotect() {
	pthread_mutex_unlock(&mLock);
}


GhwMemAllocator::~GhwMemAllocator()
{
}

GhwAllocatorImpl::GhwAllocatorImpl(u32 mode , u32 slab_size , u32 alignment)
    :mSlabSize (slab_size), mAlignment(alignment), mMode(mode), mTotalAllocSize(0)
{
//	if(mMode == GHW_MEM_ALLOC_SIMPLE) mMode = GHW_MEM_ALLOC_RETAIN_ONE;
	mDevice.setMode(mode&GHW_MEM_ALLOC_CACHED);
	mMode = mode & (~GHW_MEM_ALLOC_CACHED);
	if(mAlignment > 12) mAlignment = 12;
	if(mSlabSize == 0) mSlabSize = 1024*1024;
	count++;
}

ghw_error_e GhwAllocatorImpl::initCheck()
{
    if(mDevice.initCheck())
        return GHW_ERROR_FAIL;
    else
        return GHW_ERROR_NONE;
}

GhwAllocatorImpl::~GhwAllocatorImpl()
{
	count--;

	protect();
    GhwMemBlockNode* node = mList.getHead();
    while(node) {
        node->get()->acquire();
		mTotalAllocSize -= node->get()->getSize();
        delete node->get();
        mList.removeNode(node);
        node = mList.getHead();
    }

    mSlabSize = 0 ;
    mAlignment = 0 ;
	mMode = 0 ;
    mTotalAllocSize = 0 ;

	unprotect();
}

GhwMemHandle* GhwAllocatorImpl::alloc(u32 size, u32 alignment )
{
    if((alignment > 12) || (size == 0)) return NULL;

    protect();
    if(alignment < mAlignment) alignment = mAlignment;

    GhwMemBlockNode* node = mList.getHead();
    while(node) {
        GhwMemBlock* handle = node->get();
        GhwMemHandle* mem = handle->alloc(size,alignment);
        if(mem) {
			unprotect();
			return mem;
		}
        node = node->getNext();
    }

    u32 alignsize = size ;
    if(mMode != GHW_MEM_ALLOC_SIMPLE) {
		if(alignsize > mSlabSize) {
			u32 factor = (alignsize+mSlabSize-1)/mSlabSize;
			alignsize = factor*mSlabSize;
		}
		else {
			alignsize = mSlabSize;
		}
	}
    GhwMemBlock* handle = new GhwMemBlock(this,mDevice,alignsize);
    if(handle && ( handle->initCheck() == 0)) {
		mTotalAllocSize += alignsize;
        handle->setNode(mList.addElement(handle,mList.getCount()));
        GhwMemHandle* mem = handle->alloc(size,alignment);
        handle->release();
		unprotect();
        return mem;
    }
	unprotect();
    return NULL;
}

ghw_error_e GhwAllocatorImpl::free(GhwMemHandle* aHandle)
{
    aHandle->release();
    return GHW_ERROR_NONE;
}

ghw_error_e GhwAllocatorImpl::reset()
{
	protect();
    GhwMemBlockNode* node = mList.getHead();
    switch (mMode) {
    case GHW_MEM_ALLOC_RETAIN_ALL:
		{
    while(node) {
        node->get()->acquire();
        node->get()->reset();
        node->get()->release();
        node = node->getNext();
        }
        break;
        }
	default:
		{
			while(node) {
				if((mMode == GHW_MEM_ALLOC_RETAIN_ONE) && (mList.getCount() == 1)) {
					node->get()->acquire();
					node->get()->reset();
					node->get()->release();
					break;
				}
				node->get()->acquire();
				mTotalAllocSize -= node->get()->getSize();
				delete node->get();
				mList.removeNode(node);
				node = mList.getHead();
				}
			break;
		}
	}
	unprotect();
    return GHW_ERROR_NONE;
}

void GhwAllocatorImpl::notifyParent(GhwMemBlock* handle)
{
	if(mMode == GHW_MEM_ALLOC_RETAIN_ALL) return;
	if((mMode == GHW_MEM_ALLOC_RETAIN_ONE) && (mList.getCount() == 1)) return;

	handle->acquire();
	mTotalAllocSize -= handle->getSize();
    mList.removeNode(handle->getNode());
    delete handle;
}


ghw_error_e GhwAllocatorImpl::virt2phys(u32& ipa_addr, void* virt_addr)
{
	protect();
	unsigned char* addrin = (unsigned char*) virt_addr;
	GhwMemBlockNode* node = mList.getHead();
	while(node) {
		u32 ipa,size;
		unsigned char* addr;
		node->get()->lock(ipa,(void*&)addr,size);
		if((addrin > addr) && (addrin < (addr +size)) ) {
			ipa_addr = ipa + ((unsigned int) (addrin-addr));
			node->get()->unlock();
			unprotect();
			return GHW_ERROR_NONE;
		}
		node->get()->unlock();
		node = node->getNext();
	}

	unprotect();
	return GHW_ERROR_FAIL;
}
ghw_error_e GhwAllocatorImpl::phys2virt(u32 ipa_addr, void*& virt_addr)
{
	protect();
	unsigned int addrin = (unsigned int) ipa_addr;
	GhwMemBlockNode* node = mList.getHead();
	while(node) {
		u32 ipa,size;
		unsigned char* addr;
		node->get()->lock(ipa,(void*&)addr,size);
		if((addrin > ipa) && (addrin < (ipa +size)) ) {
			virt_addr = (void*) ( addr + ((unsigned int) (addrin-ipa)));
			node->get()->unlock();
			unprotect();
			return GHW_ERROR_NONE;
		}
		node->get()->unlock();
		node = node->getNext();
	}

	unprotect();
	return GHW_ERROR_FAIL;
}


void GhwAllocatorImpl::protect()
{
	mDevice.protect();
}

void GhwAllocatorImpl::unprotect()
{
	mDevice.unprotect();
}

ghw_error_e GhwAllocatorImpl::dump(u32 level )
{
	protect();

	LOGD("GhwAllocatorImpl Static Counters %d %d %d\n",GhwAllocatorImpl::count,GhwMemBlock::count,GhwAllocatorDevice::count);

	u32 maxFree = 0, totalFree = 0;

	GhwMemBlockNode* node = mList.getHead();
	while(node) {
		totalFree += node->get()->getFreeSize();
		u32 blockMax = node->get()->getMaxFreeSize();
		if(blockMax > maxFree) maxFree = blockMax;
		node = node->getNext();
	}

	LOGD("GhwAllocatorImpl[%x] totalDeviceAllocSize[%d] totalFree[%d] maxFree[%d] in numSlabs[%d]\n",
									this,mTotalAllocSize,totalFree,maxFree,mList.getCount());
	if (level) {
		level--;
		GhwMemBlockNode* node = mList.getHead();
		while(node) {
			node->get()->dump(level);
			node = node->getNext();
		}
	}
	unprotect();
	return GHW_ERROR_NONE;
}
};
