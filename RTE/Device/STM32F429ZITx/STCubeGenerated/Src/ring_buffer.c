/* Includes ------------------------------------------------------------------*/
#include <assert.h>
#include "ring_buffer.h"


bool RingBuffer_Init(RingBuffer *ringBuffer, char *dataBuffer, size_t dataBufferSize) 
{
	assert(ringBuffer);
	assert(dataBuffer);
	assert(dataBufferSize > 0);
	
	if ((ringBuffer) && (dataBuffer) && (dataBufferSize > 0)) {
	  
	  ringBuffer->dataBufferHead = dataBuffer;
	  ringBuffer->dataBufferSize = dataBufferSize;
	  ringBuffer->head = dataBuffer;
	  ringBuffer->tail = dataBuffer;
	
	  ringBuffer->empty = true;

	  return true;
	}
	
	return false;
}

bool RingBuffer_Clear(RingBuffer *ringBuffer)
{
	assert(ringBuffer);
	
	if (ringBuffer) {
		ringBuffer->tail = ringBuffer->dataBufferHead;
		ringBuffer->head = ringBuffer->dataBufferHead;
		ringBuffer->empty = true;
		return true;
	}
	return false;
}

bool RingBuffer_IsEmpty(const RingBuffer *ringBuffer)
{
  assert(ringBuffer);	
	    return ringBuffer->empty;
}

size_t RingBuffer_GetLen(const RingBuffer *ringBuffer)
{
	assert(ringBuffer);
	
	if (ringBuffer) {
	    int count = (ringBuffer->head - ringBuffer->tail );
	    if (ringBuffer->empty && count!=0)	                return 0;
	    else if (!ringBuffer->empty&&count==0)              return ringBuffer->dataBufferSize;
	    else if (count < 0)					                return ringBuffer->dataBufferSize + count;
	    else                                                return count;

		

	}
	return 0;
	
}

size_t RingBuffer_GetCapacity(const RingBuffer *ringBuffer)
{
	assert(ringBuffer);
	
	if (ringBuffer)
	{
		return ringBuffer->dataBufferSize;
	}
	return 0;	
}


bool RingBuffer_PutChar(RingBuffer *ringBuffer, char c)
{
	assert(ringBuffer);
	if (ringBuffer) {
		if ((ringBuffer->tail == ringBuffer->head) && !ringBuffer->empty)
		{
		    return false;
		}
		
		*ringBuffer->head = c;
        
        
        
		ringBuffer->empty = false;
		
		if (ringBuffer->head == (ringBuffer->dataBufferHead + ringBuffer->dataBufferSize -1 ))
		{
			ringBuffer->head = ringBuffer->dataBufferHead;
		}
		else
		{
			(ringBuffer->head)++;
		}
		return true;
	}
	return false;
}

bool RingBuffer_GetChar(RingBuffer *ringBuffer, char *c)
{
	assert(ringBuffer);
	assert(c);
	
  if ((ringBuffer) && (c) && !ringBuffer->empty)
	{
	  *c = *ringBuffer->tail;
	  if (ringBuffer->tail == (ringBuffer->dataBufferHead + ringBuffer->dataBufferSize - 1))
	  {
		  ringBuffer->tail = ringBuffer->dataBufferHead;
	  }
	  else
	  {
		  (ringBuffer->tail)++;
	  }
	  if (ringBuffer->tail == ringBuffer->head) 
	  {
	     ringBuffer->empty = true;
	  }
	  return true;
	}
	return false;
}
