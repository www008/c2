#ifndef DQUEUE_H
#define DQUEUE_H

#include "stdio.h"

template <typename T,int N>
class dqueue {
public:
    struct __queue__iterator {
        typedef T   value_type;
        typedef T   reference;
        typedef T*  pointer;
        typedef struct __queue__iterator self;

        int current;
        T* base;
        
        __queue__iterator() {}
        __queue__iterator(int x, T* q):current(x),base(q) {}
        //__queue__iterator(const iterator& x):current(x.current) {}
        
        bool operator==(const self&x) const { return current == x.current; }
        bool operator!=(const self&x) const { return current != x.current; }
        reference operator*() const { return base[current]; }
        pointer operator->() const{ return &base[current]; }
        self& operator++() { current = (current + 1) % N; return *this; }   // ++it
        self& operator++(int) { current = (current + 1) % N; return *this; } // it++
    };

    typedef struct __queue__iterator  iterator;
    typedef size_t      size_type;
protected:
    int front;
    int rear;
    T q[N];
public:
    dqueue():front(0), rear(0) {
    }

    bool push(const T& v) {
        if ( (rear + 1)%N == front ) { //is full
            front = (front + 1) % N;
        }
        q[rear] = v;
        rear = (rear + 1) % N;
        return true;
    }

    T pop() {
        rear = (rear - 1 + N) % N;
        T tmp = q[rear];
        return tmp;
    }
    
    T popHead() {
        T v = q[front];
        front = (front + 1) % N;
        return v;
    }
    
    T getTop() {
        return q[(rear - 1 + N) % N];
    }
    
    void setTop(const T& v) {
        q[(rear - 1 + N) % N] = v;
    }
    
    inline void clear() { 
        front = rear =0;
    }
    
    inline size_type size() const { 
        return size_type((rear - front + N) % N);
    }

    inline bool empty() const { 
        return (rear == front);
    }
    
    iterator begin() { return iterator(front, q); }
    
    iterator end() { return iterator(rear, q); }
};

struct point {
public:
    float x;
    float y;
    float z;
    point(): x(0.0f), y(0.0f), z(0.0f){ }
    point(float a, float b, float c) : x(a), y(b), z(c) {}
    point(struct point& v):x(v.x),y(v.y),z(v.z) { }
    struct point& operator= (const struct point& e) {  
        x = e.x;
        y = e.y;
        z = e.z;  
        return *this;  
    }
    //struct point operator+ (const struct point& s1, const struct point& s2) {  
    //    return struct point(s1.x+s2.x, s1.y+s2.y, s1.z+s2.z);  
    //}
    struct point& operator+= (const struct point& e) {
        x += e.x;
        y += e.y;
        z += e.z;
        return *this;  
    }
};
typedef struct point Point;

void pr( dqueue<int,40>& q ){
    char buf[500];
    int len=0;
    for(dqueue<int,40>::iterator it=q.begin(); it!=q.end(); it++) {
        int j=sprintf(buf+len," %d,",*it);
        len += j;
    }
    printf("\r\n%s\r\n",buf );
}


/***** Cov ****
Xij = SUM (Xi- Xm)*(Xj -Xn) = SUM ( Xi*Xj - Xm*Xj -Xn*Xi + XmXn)
    = SUM Xi*Xj - SUM Xm*Xj - SUM Xn*Xi + SUM XmXn
    = SUM Xi*Xj - n*Xm*Xn - m*Xm*Xn + n*Xm*Xn  (m=n)
    = SUM Xi*Xj - n*Xm*Xn
( SUM Xm*Xj = Xm * SUM Xj = Xm * n * Xn = n*Xm*Xn )
**********/
template <typename T, int N, typename Q = dqueue<T,N> >
void cov(Q& q, float cov[][3]) {
    float dsize = q.size();
    cov[0][0] = cov[0][1] = cov[0][2] = 0;
    cov[1][0] = cov[1][1] = cov[1][2] = 0;
    cov[2][0] = cov[2][1] = cov[2][2] = 0;
    Point mean(0, 0, 0);
    for(typename Q::iterator it=q.begin(); it!=q.end(); it++ ){
        //mean += *it;
        mean.x += it->x;
        mean.y += it->y;
        mean.z += it->z;
        
        cov[0][0] += (it->x * it->x);
        cov[1][1] += (it->y * it->y);
        cov[2][2] += (it->z * it->z);
        cov[0][1] += (it->x * it->y);
        cov[0][2] += (it->x * it->z);
        cov[1][2] += (it->y * it->z);
    }
    mean.x /= dsize;
    mean.y /= dsize;
    mean.z /= dsize;

    cov[0][0] = (cov[0][0] - dsize * mean.x * mean.x) / (dsize-1);
    cov[0][1] = (cov[0][1] - dsize * mean.x * mean.y) / (dsize-1);
    cov[0][2] = (cov[0][2] - dsize * mean.x * mean.z) / (dsize-1);
    cov[1][0] =  cov[0][1];
    cov[1][1] = (cov[1][1] - dsize * mean.y * mean.y) / (dsize-1);
    cov[1][2] = (cov[1][2] - dsize * mean.y * mean.z) / (dsize-1);
    cov[2][0] =  cov[0][2];
    cov[2][1] =  cov[1][2];
    cov[2][2] = (cov[2][2] - dsize * mean.z * mean.z) / (dsize-1);
    
    printf("cov(%f) = {\t%lf, %lf, %lf\r\n\t %lf, %lf, %lf\r\n\t %lf, %lf, %lf\r\n", dsize,
                    cov[0][0], cov[0][1],cov[0][2],
                    cov[1][0], cov[1][1],cov[1][2],
                    cov[2][0], cov[2][1],cov[2][2]
                    );
    printf("mean = {%lf, %lf, %lf} \r\n", mean.x, mean.y, mean.z );
}

void toCopy(double t[],float s[][3]){
    for(int i=0,k=0;i<3;i++)
        for(int j=0;j<3;j++) {
            t[k++] = s[i][j];
        }
}

#endif
