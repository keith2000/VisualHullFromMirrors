//PixeEtErrMex.cpp
//compute the ET err values for a viewVec

#include "Array2D.h"
#include "SilhouetteSet.h"
extern "C" {
#include "mex.h"
}


#define for if(0); else for // Required for Windows for-loop scoping issues.

using namespace Wm3;
using namespace std;



void CheckInput( int nlhs, mxArray *plhs[], int nrhs, const mxArray  *prhs[] )
{
	if (nrhs < 1 || nrhs > 2)
		mexErrMsgTxt("Need one/two input arg: resVec = PixeEtErrMex(viewVec) OR resVec = PixeEtErrMex(viewVec, pairlist)");
	if (nrhs==2)
		if (mxGetN(prhs[1])!=2) mexErrMsgTxt("Argument 2: PairList needs two columns.");    
}

void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray  *prhs[] )
{

	CheckInput(nlhs, plhs, nrhs, prhs);

	//	mexPrintf("stopping\n");


	try
	{
		SilhouetteSet silset( prhs[0] );


		//plhs[0]= mxCreateDoubleMatrix( silset.NumViews(),silset.NumViews(), mxREAL);	
		vector<double> resVec( 4*silset.NumViews() * (silset.NumViews()-1)  );	
		vector<double> epiInsideVec( resVec.size() );	

		Array2D<double> resArray( silset.NumViews(), silset.NumViews() );
		Array2D<double> epiInsideArray( silset.NumViews(), silset.NumViews() );


		if (nrhs == 1) { // do all combinations

			int count = 0;

			for (int viewLoopA = 0; viewLoopA < silset.NumViews(); viewLoopA++ )
				for (int viewLoopB = 0; viewLoopB < silset.NumViews(); viewLoopB++ )
				{

					if ( viewLoopA != viewLoopB )
					{

						double resData[4];


						silset.EtResiduals( viewLoopA, viewLoopB, resData );

						double rms = sqrt(
							(resData[0]*resData[0] +
							resData[1]*resData[1] +
							resData[2]*resData[2] +
							resData[3]*resData[3])/4 );


						resArray( viewLoopA, viewLoopB ) = rms;



						bool epiInside = false;
						for( int fourLoop = 0; fourLoop < 4; fourLoop++)
						{


							if ( silset.IsEpipoleInBoundRect(viewLoopA, viewLoopB ) )
							{
								epiInsideVec[count+fourLoop] = 1;
								epiInside = true;
							}
							else
								epiInsideVec[count+fourLoop] = 0;


							//epiInsideVec[count+fourLoop] = 
							//  silset.IsEpipoleInBoundRect(viewLoopA, viewLoopB ) ? 1:0;

							resVec[count+fourLoop] = resData[fourLoop];
						}


						if (epiInside)
							epiInsideArray( viewLoopA, viewLoopB ) = 1;
						else
							epiInsideArray( viewLoopA, viewLoopB ) = 0;

						count +=4;


					}
					else
					{
						resArray( viewLoopA, viewLoopB ) = 0;
						epiInsideArray( viewLoopA, viewLoopB ) = 0;
					}
				}
		} // end one arg in
		else { // if pairings are specified

			int numPairs = mxGetM(prhs[1]);

			resVec.resize(4*numPairs);
			epiInsideVec.resize(resVec.size());	

			resArray.SetAll(0);
			//    		Array2D<double> resArray( silset.NumViews(), silset.NumViews() );


			double *pairList = mxGetPr(prhs[1]);

			int count = 0;
			for (int pairInd=0; pairInd<numPairs; pairInd++) {

				int viewLoopA = pairList[pairInd]-1;
				int viewLoopB = pairList[pairInd+numPairs]-1;

				if (viewLoopA<0 || viewLoopA>silset.NumViews()-1 ||
					viewLoopB<0 || viewLoopB>silset.NumViews()-1)
					throw(string("Specified view is out of range. Check PairList."));

				if ( viewLoopA != viewLoopB )
				{

					double resData[4];


					silset.EtResiduals( viewLoopA, viewLoopB, resData );

					double rms = sqrt(
						(resData[0]*resData[0] +
						resData[1]*resData[1] +
						resData[2]*resData[2] +
						resData[3]*resData[3])/4 );


					resArray( viewLoopA, viewLoopB ) = rms;



					for( int fourLoop = 0; fourLoop < 4; fourLoop++)
					{

						epiInsideVec[count+fourLoop] = 
							silset.IsEpipoleInBoundRect(viewLoopA, viewLoopB ) ? 1:0;

						resVec[count+fourLoop] = resData[fourLoop];
					}

					count +=4;


				}
				else
				{
					resArray( viewLoopA, viewLoopB ) = 0;
				}
			}           
		}

		//now we need to replace the epipole inside cases with values that will
		//leave the rms  as if they were removed
		//doing this instead of removing them keeps the vector length
		//the same so Levenberg-Marquardt doesn't break

		double sumSqSoFar = 0;
		int numValidSoFar = 0;
		for( int resLoop = 0; resLoop < resVec.size(); resLoop++ )
		{
			if ( 0 == epiInsideVec[resLoop] )
			{
				numValidSoFar++;
				sumSqSoFar = sumSqSoFar + resVec[resLoop]*resVec[resLoop];
			}

		}

		//here we replace the invalid residuals
		for( int resLoop = 0; resLoop < resVec.size(); resLoop++ )
		{
			if ( 0 < epiInsideVec[resLoop] )
			{
				resVec[resLoop] = sqrt(sumSqSoFar/numValidSoFar);
			}

		}



		plhs[0]= mxCreateDoubleMatrix( 1, resVec.size(), mxREAL);	

		memcpy( mxGetPr(plhs[0]) , 
			&(resVec[0]), resVec.size() * sizeof(double) );


		plhs[1]= mxCreateDoubleMatrix( 1, epiInsideVec.size(), mxREAL);	

		memcpy( mxGetPr(plhs[1]) , 
			&(epiInsideVec[0]), epiInsideVec.size() * sizeof(double) );


		plhs[2]= mxCreateDoubleMatrix( silset.NumViews(), 
			silset.NumViews(), mxREAL);	

		resArray.CopyData( mxGetPr(plhs[2]) );

		plhs[3]= mxCreateDoubleMatrix( silset.NumViews(), 
			silset.NumViews(), mxREAL);	

		epiInsideArray.CopyData( mxGetPr(plhs[3]) );




	}
	catch(  string errStr )
	{
		mexPrintf("error: %s\n",errStr.c_str() );			
	}
	catch ( ... )
	{
		mexPrintf("Caught an unspecified error" );
	}


}



