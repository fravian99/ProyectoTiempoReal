
typedef struct {
    char * inner_buffer;
    int size;
    int read_ptr;
    int write_ptr;
} Buffer;

Buffer buffer_init(char *, int);
int buffer_is_full(Buffer *);
int buffer_is_empty(Buffer *);
int buffer_enque(Buffer*, char);
int buffer_deque(Buffer*, char*);
int buffer_insert_element(Buffer *, char *);
int buffer_cpy_element(Buffer *, char *);