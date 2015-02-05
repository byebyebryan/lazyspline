using UnityEngine;
using UnityEditor;
using System.Collections;
using System.Collections.Generic;

[System.Serializable]
public struct WayPoint
{
	public int id;
	public Vector3 position;
	public Quaternion rotation;
	
	public WayPoint (int _id, Vector3 _position, Quaternion _rotation)
	{
		id = _id;
		position = _position;
		rotation = _rotation;
	}
}

[System.Serializable]
public struct FirstPassResult
{
	public Transform[] controlPoints;
	public float tension;
	public float bias;
	public float[] firstPassT;
	public float[] firstPassAL;
	public float lastT;
	public float lastAL;
}

[System.Serializable]
public class Spline
{
	public FirstPassResult firstPassRes;
	public WayPoint[] resPoints;
	public WayPoint[] reducedPoints;
	
	public void FirstPassSampling(List<Transform> controlPoints, float firstPassDensity, float tension, float bias)
	{
		firstPassRes = SplineMath.FirstPassSampling(controlPoints, firstPassDensity, tension, bias);
	}
	
	public WayPoint[] GetUniformWayPoints(float density)
	{	
		resPoints = SplineMath.GetUniformWayPoints(firstPassRes, density);
		return resPoints;
	}
	
	public WayPoint[] GetUniformWayPoints(List<Transform> controlPoints, float firstPassDensity, float tension, float bias, float density)
	{
		firstPassRes = SplineMath.FirstPassSampling(controlPoints, firstPassDensity, tension, bias);
		resPoints = SplineMath.GetUniformWayPoints(firstPassRes, density);
		return resPoints;
	}
	
	public WayPoint[] GetReducedWayPoints(float cutAngle)
	{
		reducedPoints = SplineMath.GetReducedWayPoints(resPoints, cutAngle);
		return reducedPoints;
	}
	
	public WayPoint GetWayPoint(float targetAL)
	{
		return SplineMath.GetWayPoint(firstPassRes, targetAL);
	}
}

[System.Serializable]
public class SplineMover
{
	public Transform transform;
	public Transform dummy;
	public Spline spline;
	public float currentAL;
	WayPoint currentWP;
	bool reverse;
	public Vector3 offSet;
	
	public static float gravity = -9.81f;
	public bool useGravity;
	
	public SplineMover(Transform _transform, Spline _spline)
	{
		transform = _transform;
		dummy = null;
		spline = _spline;
		currentAL = 0.0f;
		reverse = false;
		useGravity = true;
	}
	
	public SplineMover(Transform _transform, Transform _dummy, Spline _spline)
	{
		transform = _transform;
		dummy = _dummy;
		spline = _spline;
		currentAL = 0.0f;
		reverse = false;
		useGravity = true;
	}
	
	public void Init()
	{
		currentWP = spline.GetWayPoint(currentAL);
		transform.position = currentWP.position;
		transform.rotation = currentWP.rotation;
		
		if (dummy)
		{
			dummy.position = currentWP.position;
			dummy.rotation = currentWP.rotation;
		}
	}
	
	public void Reset(float targetAL)
	{
		currentAL = targetAL;
		Init();
	}
	
	public void SetForward()
	{
		if (reverse)
			reverse = false;
	}
	
	public void SetReverse()
	{
		if (!reverse)
			reverse = true;
	}
	
	public Vector3 GetOffSet(SplineMover other)
	{
		Vector3 res = other.offSet - offSet;
		res.z = other.currentAL - currentAL;
		return res;
	}
	
	public void Move(Vector3 delta)
	{
		if (!reverse)
			currentAL += delta.z;
		else
			currentAL -= delta.z;
		
		delta.z = 0.0f;
		
		WayPoint nextWP = spline.GetWayPoint(currentAL);
		
		if (dummy)
		{
			dummy.position = nextWP.position;
			dummy.rotation = nextWP.rotation;
		}
		
		offSet = Quaternion.Inverse(currentWP.rotation) * (transform.position - currentWP.position);
		offSet.z = 0.0f;
		
		if (reverse)
		{
			//offSet.x = - offSet.x;
			delta.x = -delta.x;
			transform.rotation = nextWP.rotation * Quaternion.LookRotation(-Vector3.forward, Vector3.up);
		}
		else
		{
			transform.rotation = nextWP.rotation;
		}
		
		transform.position = nextWP.position + nextWP.rotation * (offSet+delta);
		
		if(useGravity)
		{
			float floor = GetFloor((offSet+delta).x);
			float y = transform.position.y;
			float resY;
			
			if(y<floor)
			{
				resY = floor;
			}
			else
			{
				resY = y + 0.5f * gravity * Time.deltaTime * Time.deltaTime;
			}
			
			transform.position = new Vector3(transform.position.x, resY, transform.position.z);
		}
			
		currentWP = nextWP;
	}
	
	float GetFloor(float offSetX)
	{
		return 0.0f;
	}
}

public class SplineMath {
	
	static int MinSamplingDivision = 2;
	static int MaxSamplineDivision = 20000;
	
	public static void GetParameters(Transform t0, Transform t1, Transform t2, Transform t3, float d, float c, float b, out float r1, out float r2, out int n, out float dt, out Quaternion rotZ)
	{
		float d01 = Vector3.Distance(t0.position,t1.position);
		float d12 = Vector3.Distance(t1.position,t2.position);
		float d23 = Vector3.Distance(t2.position,t3.position);
		
		r1 = c*((1-b)*d01 + b*d12);
		r2 = c*(b*d12+(1-b)*d23);
		
		n = Mathf.Clamp((int)(d12*d),MinSamplingDivision,MaxSamplineDivision);
		dt = 1.0f/(float)n;
		
		Quaternion forwardTurn = Quaternion.FromToRotation(t1.forward,t2.forward);
		rotZ = Quaternion.FromToRotation(forwardTurn * t1.up, t2.up);
	}	
	
	public static Vector3 GetPositionOnSpline(Transform t0, Transform t1, Transform t2, Transform t3, float r1, float r2, float t)
	{	
		return CubicHermitePosition(t1.position,t1.forward*r1,t2.position,t2.forward*r2,t);
	}
	
	public static Quaternion GetRotationOnSpline(Transform t0, Transform t1, Transform t2, Transform t3, float r1, float r2, Quaternion rotationZ , float t)
	{
		Vector3 tangent = CubicHermiteTangent(t1.position,t1.forward*r1,t2.position,t2.forward*r2,t);
		Quaternion forwardTurn = Quaternion.FromToRotation(t1.forward,tangent);
		Vector3 up = forwardTurn * t1.up;
		up = Quaternion.Slerp(Quaternion.identity, rotationZ, t) * up;
		
		return Quaternion.LookRotation(tangent,up);
	}
	
	public static Vector3 CubicHermitePosition(Vector3 p0, Vector3 m0, Vector3 p1, Vector3 m1, float t)
	{
		float t2 = t * t;
		float t3 = t2 * t;
		
		float h1 = 2*t3 - 3*t2 + 1;
		float h2 = t3 - 2*t2 + t;
		float h3 = -2*t3 + 3*t2;
		float h4 = t3 - t2;
		
		return h1 * p0 + h2*m0 + h3*p1 +h4*m1;
	}
	
	public static Vector3 CubicHermiteTangent(Vector3 p0, Vector3 m0, Vector3 p1, Vector3 m1, float t)
	{
		float t2 = t * t;
		
		float h1 = 6*t2 - 6*t;
		float h2 = 3*t2 - 4*t + 1;
		float h3 = -6*t2 + 6*t;
		float h4 = 3*t2 - 2*t;
		
		return (h1 * p0 + h2*m0 + h3*p1 +h4*m1).normalized;
	}
	
	public static int LoopIndex(int index, int count)
	{
		while (index < 0) index += count;
		while (index >= count) index -= count;
		return index;
	}
	
	public static FirstPassResult FirstPassSampling(List<Transform> wayPoints, float density, float tension, float bias)
	{
		FirstPassResult res = new FirstPassResult();
		res.controlPoints = wayPoints.ToArray();
		res.tension = tension;
		res.bias = tension;
		
		//int count = wayPoints.Count;
			
		List<float> resT = new List<float>();
		List<float> resAL = new List<float>();
		
		float currentT = 0;
		float currentAL = 0;
		
		for (int i=0;i<wayPoints.Count-3;i++)
		{
			Transform t0 = wayPoints[i];
			Transform t1 = wayPoints[i+1];
			Transform t2 = wayPoints[i+2];
			Transform t3 = wayPoints[i+3];
			
			float r1;
			float r2;
			int n;
			float dt;
			Quaternion rotZ;
			
			Vector3 prePos = t1.position;
			resT.Add(currentT);
			resAL.Add(currentAL);
			
			GetParameters(t0,t1,t2,t3,density,tension,bias,out r1,out r2,out n,out dt,out rotZ);
			for (int j=1;j<n;j++)
			{
				Vector3 newPos = GetPositionOnSpline(t0,t1,t2,t3,r1,r2,j*dt);
				currentT += dt;
				currentAL += Vector3.Distance(prePos,newPos);
				resT.Add(currentT);
				resAL.Add(currentAL);
				prePos = newPos;
			}
			currentT = i+1;
			currentAL += Vector3.Distance(prePos, t2.position);
		}
		
		resT.Add(currentT);
		resAL.Add(currentAL);
		
		res.lastT = currentT;
		res.lastAL = currentAL;
		
		res.firstPassT = resT.ToArray();
		res.firstPassAL = resAL.ToArray();
		
		return res;
	}
	
	public static float[] GetUniformT(FirstPassResult firstPassRes, float density)
	{
		int count = firstPassRes.firstPassT.Length;
		List<float> resT = new List<float>();
		
		float deltaAL = 1.0f / density;
		
		resT.Add(0.0f);
		
		int queryIndex = 1;
		float targetAL = queryIndex * deltaAL;
		
		while (queryIndex < count -1 && targetAL <= firstPassRes.firstPassAL[count-1])
		{
			float queryT = firstPassRes.firstPassT[queryIndex];
			float forwardT = firstPassRes.firstPassT[queryIndex+1];
			
			float queryAL = firstPassRes.firstPassAL[queryIndex];
			float forwardAL = firstPassRes.firstPassAL[queryIndex+1];
			
			if(queryAL <= targetAL && forwardAL >= targetAL)
			{
				float t = queryT + (forwardT-queryT)*(targetAL-queryAL)/(forwardAL-queryAL);
				resT.Add(t);
				targetAL += deltaAL;
			}
			else
			{
				queryIndex ++;
			}
		}
		
		return resT.ToArray();
	}
	
	public static float GetT(FirstPassResult firstPassRes, float targetAL)
	{
		if (targetAL<=0.0f)
			return 0.0f;
	
		int count = firstPassRes.firstPassT.Length;
		
		int queryIndex = 1;
		
		while (queryIndex < count -1 && targetAL <= firstPassRes.lastAL)
		{
			float queryT = firstPassRes.firstPassT[queryIndex];
			float forwardT = firstPassRes.firstPassT[queryIndex+1];
			
			float queryAL = firstPassRes.firstPassAL[queryIndex];
			float forwardAL = firstPassRes.firstPassAL[queryIndex+1];
			
			if(queryAL <= targetAL && forwardAL >= targetAL)
			{
				float t = queryT + (forwardT-queryT)*(targetAL-queryAL)/(forwardAL-queryAL);
				return t;
			}
			else
			{
				queryIndex ++;
			}
		}
		
		return firstPassRes.lastT;
	}
	
	public static WayPoint[] GetUniformWayPoints(FirstPassResult firstPassRes, float density)
	{
		float[] uniformT = GetUniformT(firstPassRes, density);
		
		List<WayPoint> res = new List<WayPoint>();
		
		int currentCP = 0;
		
		Transform t0 = firstPassRes.controlPoints[currentCP];
		Transform t1 = firstPassRes.controlPoints[currentCP+1];
		Transform t2 = firstPassRes.controlPoints[currentCP+2];
		Transform t3 = firstPassRes.controlPoints[currentCP+3];
		
		float r1;
		float r2;
		int n;
		float dt;
		Quaternion rotZ;
		
		GetParameters(t0,t1,t2,t3,density,firstPassRes.tension,firstPassRes.bias,out r1,out r2,out n,out dt,out rotZ);
		
		for (int i=0;i<uniformT.Length;i++)
		{
			float t = uniformT[i]-currentCP;
			if (t>1)
			{
				currentCP ++;
				
				if(currentCP >= firstPassRes.controlPoints.Length -3)
				{
					return res.ToArray();
				}
				
				t0 = firstPassRes.controlPoints[currentCP];
				t1 = firstPassRes.controlPoints[currentCP+1];
				t2 = firstPassRes.controlPoints[currentCP+2];
				t3 = firstPassRes.controlPoints[currentCP+3];
				
				GetParameters(t0,t1,t2,t3,density,firstPassRes.tension,firstPassRes.bias,out r1,out r2,out n,out dt,out rotZ);
				t-=1.0f;
			}
			
			Vector3 pos = GetPositionOnSpline(t0, t1, t2, t3, r1, r2, t);
			Quaternion rot = GetRotationOnSpline(t0, t1, t2, t3, r1, r2, rotZ, t);
			
			res.Add(new WayPoint(i,pos,rot));
		}
		
		res.Add (new WayPoint(uniformT.Length,firstPassRes.controlPoints[firstPassRes.controlPoints.Length-2].position,firstPassRes.controlPoints[firstPassRes.controlPoints.Length-2].rotation));
		
		return res.ToArray();
	}
	
	public static WayPoint GetWayPoint(FirstPassResult firstPassRes, float targetAL)
	{
		float uniformT = GetT(firstPassRes, targetAL);
		
		int currentCP = (int) uniformT;
		float t = uniformT - currentCP;
		
		if (currentCP >= firstPassRes.controlPoints.Length-3)
		{
			currentCP --;
			t+=1.0f;
		}
		
		Transform t0 = firstPassRes.controlPoints[currentCP];
		Transform t1 = firstPassRes.controlPoints[currentCP+1];
		Transform t2 = firstPassRes.controlPoints[currentCP+2];
		Transform t3 = firstPassRes.controlPoints[currentCP+3];
		
		float r1;
		float r2;
		int n;
		float dt;
		Quaternion rotZ;
		
		GetParameters(t0,t1,t2,t3,1,firstPassRes.tension,firstPassRes.bias,out r1,out r2,out n,out dt,out rotZ);
		
		Vector3 pos = GetPositionOnSpline(t0, t1, t2, t3, r1, r2, t);
		Quaternion rot = GetRotationOnSpline(t0, t1, t2, t3, r1, r2, rotZ, t);
		
		return new WayPoint(-1, pos, rot);
	}
	
	public static WayPoint[] GetReducedWayPoints(WayPoint[] srcWayPoints, float cutAngle)
	{
		List<WayPoint> res = new List<WayPoint>();
		
		WayPoint prevWp = srcWayPoints[0];
		WayPoint nextWp;
		
		res.Add(prevWp);
		
		int queryIndex = 1;
		
		while(queryIndex < srcWayPoints.Length)
		{
			nextWp = srcWayPoints[queryIndex];
			
			if (Quaternion.Angle(prevWp.rotation, nextWp.rotation)<cutAngle && queryIndex <srcWayPoints.Length-1)
			{
				queryIndex ++;
			}
			else
			{
				res.Add(nextWp);
				prevWp = nextWp;
				queryIndex ++;
			}
		}
		
		return res.ToArray();
	}
}
