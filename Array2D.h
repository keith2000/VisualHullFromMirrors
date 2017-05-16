#ifndef ARRAY2D_H
#define ARRAY2D_H

#include <memory.h>

template <typename  T>
class Array2D
{
public:

	Array2D( const int numRows,  const int numCols );
	Array2D();
	~Array2D();

	void Resize( const int numRows,  const int numCols );


	T& operator()(const int r, const int c) {return data[c*numberOfRows+r]; } // Returns array element
	T operator()(const int r, const int c) const {return data[c*numberOfRows+r]; } // Returns array element

	
	void CopyData( T* dest ) const;
	void CopyFrom( T* src ) const;
	void CopyTo( T* dest ) const;

	void SetAll( const T & val );

	//void CopyData( T* dest ) const{
	//	 memcpy(dest, data, 
	//		 numberOfRows * numberOfColumns * sizeof(T) );
	//}


	unsigned int NumRows() const{ return numberOfRows;}
	unsigned int NumColumns() const{ return numberOfColumns;}

	
private:
	unsigned int numberOfRows;
    unsigned int numberOfColumns;
	T* data;
	
};


/*/
template <typename  T>
Array2D<T>::Array2D( const int numRows,  const int numCols )
      : numberOfRows( numRows )
      , numberOfColumns( numCols )
      , data( new T[ numRows * numCols ] )
   {
   }


template <typename  T>
Array2D<T>::~Array2D()
{
	delete [] data;
}
/*/

//If I put these in the cpp file then it doesn't work:
//I suppose templates have to go in the header file

template <typename  T>
Array2D<T>::Array2D( const int numRows,  const int numCols )
      : numberOfRows( numRows )
      , numberOfColumns( numCols )
      , data( new T[ numRows * numCols ] )
   {
   }


template <typename  T>
Array2D<T>::Array2D()
      : numberOfRows( 0 )
      , numberOfColumns( 0 )
      , data( 0 )
   {
   }

template <typename  T>
void Array2D<T>::Resize( const int numRows,  const int numCols )      
{
	
	if ( data != 0 )	
		delete [] data; //delete existing data

	numberOfRows = numRows;
	numberOfColumns = numCols;
	data = new T[ numRows * numCols ];
}



template <typename  T>
Array2D<T>::~Array2D()
{
	delete [] data;
}

template <typename  T>
void Array2D<T>::CopyData( T* dest ) const
{
	memcpy(dest, data, 
		numberOfRows * numberOfColumns * sizeof(T) );
}


template <typename  T>
void Array2D<T>::CopyTo( T* dest ) const
{
	memcpy(dest, data, 
		numberOfRows * numberOfColumns * sizeof(T) );
}


template <typename  T>
void Array2D<T>::CopyFrom( T* src ) const
{
	memcpy(data, src, 
		numberOfRows * numberOfColumns * sizeof(T) );
}




template <typename  T>
void Array2D<T>::SetAll( const T & val ) 
{

	for( unsigned int rr = 0; rr < NumRows(); rr++ )
		for( unsigned int cc = 0; cc < NumColumns(); cc++ )
			data[cc*numberOfRows+rr] = val;

	
}



#endif 