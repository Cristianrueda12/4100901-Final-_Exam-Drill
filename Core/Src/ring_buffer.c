#include "ring_buffer.h"

uint16_t lap_counter_head=0;

uint16_t counter_size_head=0;

void ring_buffer_init(ring_buffer_t *rb, uint8_t  *mem_add, uint8_t cap)
{
	rb->buffer =mem_add;
	rb->capacity = cap;
	ring_buffer_reset(rb);

}

uint8_t ring_buffer_size(ring_buffer_t *rb)
{
	uint8_t size = 0;
	if (rb->head >= rb->tail) {
		size = rb->head - rb->tail;
	} else if (rb->is_full == 0) {
		size = (rb->capacity - rb->tail) + rb->head;
	} else {
		size = rb->capacity;
	}
	return size;
}

uint8_t ring_buffer_is_empty(ring_buffer_t *rb){


	return((rb->head== rb->tail) && (rb->is_full==0)) ? 1: 0;


}



uint8_t ring_buffer_is_full(ring_buffer_t *rb){

	if( rb->is_full!=0){
		return 1; //esta lleno
	}else{
		return 0;// No esta lleno
	}


}

void ring_buffer_reset(ring_buffer_t *rb)
{   rb->head = 0;
    rb->tail = 0;
    rb->is_full = 0;

}

void ring_buffer_write(ring_buffer_t *rb, uint8_t data)
{
	rb->buffer[rb->head] = data;
	rb->head = rb->head + 1;


        if(rb->head>= rb->capacity){ // Si llega al final de la memoria
        	rb->head = 0;
			lap_counter_head=2; // hace el conteo de la primer vuelta

		}
		if(rb->is_full!= 0){ // Si se pierden los datos
			rb->tail =rb->tail +1;
		}
		if(rb->tail >= rb->capacity){ // Si la cola llega al final de la memoria
			rb->tail=0;
		}
		if(rb->head == rb->tail){ // Si la cabeza alcanza la cola
			rb->is_full=1;
			counter_size_head++;// cuenta las posiciones que se corrio la cabeza despues de la primer vuelta.

        }
 }

uint8_t ring_buffer_read(ring_buffer_t *rb, uint8_t *byte)
{

	if (( rb->is_full!=0)||(rb->head != rb->tail))
	{

		*byte = rb->buffer[rb->tail];
		rb->tail = rb->tail +1;
		if(rb->tail >= rb->capacity){
			rb->tail =0;
		}
	    rb->is_full =0;
	  return 1;
    }
	  return 0;
}
