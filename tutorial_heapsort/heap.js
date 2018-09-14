/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Heap Sort Stencil | JavaScript support functions

    Quick JavaScript Code-by-Example Tutorial 
     
    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License 

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/


// create empty object 
minheaper = {}; 

// define insert function for min binary heap
function minheap_insert(heap, new_element) {
    heap.push(new_element);
    x = heap.length-1;
    while (x !== 0) {
        y = Math.floor((x-1)/2);
        if (heap[x] < heap[y]) {
           a = heap[y];
           heap[y] = heap[x];
           heap[x] = a;
        }
        x = y;
    }
    // STENCIL: implement your min binary heap insert operation
}

// assign insert function within minheaper object
minheaper.insert = minheap_insert;
/* Note: because the minheap_insert function is an object, we can assign 
      a reference to the function within the minheap object, which can be called
      as minheap.insert
*/

// define extract function for min binary heap
function minheap_extract(heap) {
    if (heap.length>1){
        b = heap[0];
        heap[0]=heap[heap.length-1];
        heap[heap.length-1] = b;
        x = 0;
        y = 1;
        while (y+2 < heap.length){
            if (heap[y] < heap[y+1]){
                if (heap[y] < heap[x]){
                    a = heap[x];
                    heap[x] = heap[y];
                    heap[y] = a;
                    x = y;
                    y = 2*x+1;
                    }
                else {
                    break
                }    
                }
            else {
                if (heap[y+1] < heap[x]){
                    a = heap[x];
                    heap[x] = heap[y+1];
                    heap[y+1] = a;
                    x = y+1;
                    y = 2*x+1;
                }
                else {
                    break
                }
            }
        }
        if (y+2 === heap.length){
            if(heap[y] < heap[x]){
                a = heap[x];
                heap[x] = heap[y];
                heap[y] = a;
            }
        }
        return heap.pop()
    }
    else if (heap.length === 1){
        return heap.pop();
    }
    // STENCIL: implement your min binary heap extract operation
}

// assign extract function within minheaper object
minheaper.extract = minheap_extract;
    // STENCIL: ensure extract method is within minheaper object






