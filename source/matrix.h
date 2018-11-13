#pragma once

#include "vec.h"

namespace BaluLib
{

	template<class T, int Size>
	struct TPlane;

	template<class T, int Size>
	class TMatrix
	{
		friend class TMatrix<T, Size + 1>;
	public:
		enum
		{
			high = Size - 1
		};
	private:
		TVec<T, Size> m[Size]; //m[col][row]
		// [0,0][1,0][2,0]
		// [0,1][1,1][2,1]
		// [0,2][1,2][2,2] 
	public:
		TMatrix(){}
		void SetIdentity()
		{
			for (int i = 0; i < Size; i++)
				for (int k = 0; k < Size; k++)
					m[i][k] = (i == k ? (T)1 : (T)0);
		}
		TMatrix(const TVec<T, 2>& v0, const TVec<T, 2>& v1)
		{
			static_assert(Size == 2, "only 2d matrix constructor");
			m[0] = v0;
			m[1] = v1;
		}
		TMatrix(const TVec<T, 3>& v0, const TVec<T, 3>& v1, const TVec<T, 3>& v2)
		{
			static_assert(Size == 3, "only 3d matrix constructor");
			m[0] = v0;
			m[1] = v1;
			m[2] = v2;
		}
		TMatrix(const TVec<T, 4>& v0, const TVec<T, 4>& v1, const TVec<T, 4>& v2, const TVec<T, 4>& v3)
		{
			static_assert(Size == 4, "only 4d matrix supported");
			m[0] = v0;
			m[1] = v1;
			m[2] = v2;
			m[3] = v3;
		}
		TMatrix(
			T v0, T v2,
			T v1, T v3)
		{
			static_assert(Size == 2, "only 2d matrix supported");
			m[0][0] = v0; m[1][0] = v2;
			m[0][1] = v1; m[1][1] = v3;
		}
		TMatrix(
			T v0, T v3, T v6,
			T v1, T v4, T v7,
			T v2, T v5, T v8)
		{
			static_assert(Size == 3, "only 3d matrix supported");
			m[0][0] = v0; m[1][0] = v3; m[2][0] = v6;
			m[0][1] = v1; m[1][1] = v4; m[2][1] = v7;
			m[0][2] = v2; m[1][2] = v5; m[2][2] = v8;
		}
		TMatrix(
			T v0, T v4, T v8, T v12,
			T v1, T v5, T v9, T v13,
			T v2, T v6, T v10, T v14,
			T v3, T v7, T v11, T v15)
		{
			static_assert(Size == 4, "only 4d matrix supported");
			m[0][0] = v0; m[1][0] = v4; m[2][0] = v8; m[3][0] = v12;
			m[0][1] = v1; m[1][1] = v5; m[2][1] = v9; m[3][1] = v13;
			m[0][2] = v2; m[1][2] = v6; m[2][2] = v10; m[3][2] = v14;
			m[0][3] = v3; m[1][3] = v7; m[2][3] = v11; m[3][3] = v15;
		}
		TMatrix(const TMatrix<T, Size - 1>& matrix, const TVec<T, Size - 1>& off)
		{
			for (int i = 0; i < Size - 1; i++)
			{
				m[i] = TVec<T, Size>(matrix[i], 0);
			}
			m[high] = TVec<T, Size>(off, 1);
		}

		TVec<T, Size> TransMul(const TVec<T, Size>& v0)const
		{
			static_assert(Size >= 2 && Size <= 4, "only 2d,3d,4d matrix supported");
			TVec<T, Size> t;
			if (Size >= 1)t[0] = m[0] * v0;
			if (Size >= 2)t[1] = m[1] * v0;
			if (Size >= 3)t[2] = m[2] * v0;
			if (Size >= 4)t[3] = m[3] * v0;
			return t;
		}
		TVec<T, Size> AbsMul(const TVec<T, Size>& v0)const
		{
			static_assert(Size >= 2 && Size <= 4, "only 2d,3d,4d matrix supported");
			TVec<T, Size> t;
			if (Size == 2)
			{
				t[0] = v0[0] * abs(m[0][0]) + v0[1] * abs(m[1][0]);
				t[1] = v0[0] * abs(m[0][1]) + v0[1] * abs(m[1][1]);
			}
			if (Size == 3)
			{
				t[0] = v0[0] * abs(m[0][0]) + v0[1] * abs(m[1][0]) + v0[2] * abs(m[2][0]);
				t[1] = v0[0] * abs(m[0][1]) + v0[1] * abs(m[1][1]) + v0[2] * abs(m[2][1]);
				t[2] = v0[0] * abs(m[0][2]) + v0[1] * abs(m[1][2]) + v0[2] * abs(m[2][2]);
			}
			if (Size == 4)
			{
				t[0] = v0[0] * abs(m[0][0]) + v0[1] * abs(m[1][0]) + v0[2] * abs(m[2][0]) + v0[3] * abs(m[3][0]);
				t[1] = v0[0] * abs(m[0][1]) + v0[1] * abs(m[1][1]) + v0[2] * abs(m[2][1]) + v0[3] * abs(m[3][1]);
				t[2] = v0[0] * abs(m[0][2]) + v0[1] * abs(m[1][2]) + v0[2] * abs(m[2][2]) + v0[3] * abs(m[3][2]);
				t[3] = v0[0] * abs(m[0][3]) + v0[1] * abs(m[1][3]) + v0[2] * abs(m[2][3]) + v0[3] * abs(m[3][3]);
			}
			return t;
		}

		static TMatrix RotateX(const T angle)
		{
			static_assert(Size >= 3 && Size <= 4, "only 2d,3d,4d matrix supported");
			TMatrix<T, Size> t;
			T cosA = cos(angle), sinA = sin(angle);
			if (Size >= 3)
			{
				t[0][0] = 1;		t[0][1] = 0;		t[0][2] = 0;
				t[1][0] = 0;		t[1][1] = cosA;	t[1][2] = sinA;
				t[2][0] = 0;		t[2][1] = -sinA;	t[2][2] = cosA;
				if (Size == 4)
				{
					t[3][0] = 0;		t[3][1] = 0;		t[3][2] = 0;
					t[0][3] = 0;		t[1][3] = 0;		t[2][3] = 0; t[3][3] = 1;
				}
			}
			return t;
		}
		static TMatrix RotateY(const T angle)
		{
			static_assert(Size >= 3 && Size <= 4, "only 2d,3d,4d matrix supported");
			TMatrix<T, Size> t;
			T cosA = cos(angle), sinA = sin(angle);
			if (Size >= 3)
			{
				t[0][0] = cosA;	t[0][1] = 0;		t[0][2] = sinA;
				t[1][0] = 0;		t[1][1] = 1;		t[1][2] = 0;
				t[2][0] = -sinA;	t[2][1] = 0;		t[2][2] = cosA;
				if (Size == 4)
				{
					t[3][0] = 0;		t[3][1] = 0;		t[3][2] = 0;
					t[0][3] = 0;		t[1][3] = 0;		t[2][3] = 0; t[3][3] = 1;
				}
			}
			return t;
		}
		static TMatrix RotateZ(const T angle)
		{
			static_assert(Size >= 2 && Size <= 4, "only 2d,3d,4d matrix supported");
			TMatrix<T, Size> t;
			T cosA = cos(angle), sinA = sin(angle);
			if (Size >= 2)
			{
				t[0][0] = cosA;	t[0][1] = sinA;
				t[1][0] = -sinA;	t[1][1] = cosA;
				if (Size >= 3)
				{
					t[0][2] = 0;
					t[1][2] = 0;
					t[2][0] = 0;
					t[2][1] = 0;
					t[2][2] = 0;
					if (Size == 4)
					{
						t[3][0] = 0;		t[3][1] = 0;		t[3][2] = 0;
						t[0][3] = 0;		t[1][3] = 0;		t[2][3] = 0; t[3][3] = 1;
					}
				}
			}
			return t;
		}

		static TMatrix<T, Size> RotateXYZ(const T angle_x, const T angle_y, const T angle_z)
		{
			static_assert(Size >= 2 && Size <= 4, "only 2d,3d,4d matrix supported");
			TMatrix<T, Size> mat;
			T A = cos(angle_x);
			T B = sin(angle_x);
			T C = cos(angle_y);
			T D = sin(angle_y);
			T E = cos(angle_z);
			T F = sin(angle_z);
			T AD = A * D;
			T BD = B * D;

			mat[0][0] = C * E;
			mat[1][0] = -C * F;
			mat[2][0] = -D;
			mat[0][1] = -BD * E + A * F;
			mat[1][1] = BD * F + A * E;
			mat[2][1] = -B * C;
			mat[0][2] = AD * E + B * F;
			mat[1][2] = -AD * F + B * E;
			mat[2][2] = A * C;

			if (Size == 4)
			{
				mat[0][3] = 0;
				mat[1][3] = 0;
				mat[2][3] = 0;
				mat[3][0] = 0;
				mat[3][1] = 0;
				mat[3][2] = 0;
				mat[3][3] = 1;
			}
			return mat;
		}

		static TMatrix GetIdentity()
		{
			static_assert(Size >= 2 && Size <= 4, "only 2d,3d,4d matrix supported");
			TMatrix<T, Size> t;
			t.SetIdentity();
			return t;
		}

		TMatrix GetTransposed()const
		{
			static_assert(Size >= 2 && Size <= 4, "only 2d,3d,4d matrix supported");
			TMatrix<T, Size> t;
			if (Size == 2)
			{
				t.m[0][0] = m[0][0];	t.m[0][1] = m[1][0];
				t.m[1][0] = m[0][1];	t.m[1][1] = m[1][1];
			}
			if (Size == 3)
			{
				t.m[0][0] = m[0][0];	t.m[0][1] = m[1][0];	t.m[0][2] = m[2][0];
				t.m[1][0] = m[0][1];	t.m[1][1] = m[1][1];	t.m[1][2] = m[2][1];
				t.m[2][0] = m[0][1];	t.m[2][1] = m[1][2];	t.m[2][2] = m[2][2];
			}
			if (Size == 4)
			{
				t.m[0][0] = m[0][0];	t.m[0][1] = m[1][0];	t.m[0][2] = m[2][0];	t.m[0][3] = m[3][0];
				t.m[1][0] = m[0][1];	t.m[1][1] = m[1][1];	t.m[1][2] = m[2][1];	t.m[1][3] = m[3][1];
				t.m[2][0] = m[0][2];	t.m[2][1] = m[1][2];	t.m[2][2] = m[2][2];	t.m[2][3] = m[3][2];
				t.m[3][0] = m[0][3];	t.m[3][1] = m[1][3];	t.m[3][2] = m[2][3];	t.m[3][3] = m[3][2];
			}
			return t;
		}
		void Transpose()
		{
			static_assert(Size >= 2 && Size <= 4, "only 2d,3d,4d matrix supported");
			T temp;
#define swap(c,r) temp=m[c][r];m[c][r]=m[r][c];m[r][c]=temp;
			if (Size >= 2)
			{
				swap(0, 1);
			}
			if (Size >= 3)
			{
				swap(0, 2);
				swap(1, 2);
			}
			if (Size >= 4)
			{
				swap(0, 3);
				swap(1, 3);
				swap(2, 3);
			}
#undef swap
		}

		friend TMatrix operator*(const TMatrix &m, const T val)
		{
			TMatrix t;
			for (int i = 0; i < Size; i++)
				t[i] = m[i] * val;
			return t;
		}
		void operator*=(const T val)
		{
			for (int i = 0; i < Size; i++)
				m[i] *= val;
		}

		friend bool operator==(const TMatrix &m0, const TMatrix &m1)
		{
			for (int i = 0; i < Size; i++)
				if (m0[i] != m1[i])
					return false;
			return true;
		}
		friend bool operator!=(const TMatrix &m0, const TMatrix &m1)
		{
			for (int i = 0; i < Size; i++)
				if (m0[i] != m1[i])
					return true;
			return false;
		}
		friend TVec<T, Size> operator*(const TMatrix &_m, const TVec<T, Size>& _v0)
		{
			static_assert(Size >= 2 && Size <= 4, "only 2d,3d,4d matrix supported");
			TVec<T, Size> _t;
			T* m = (T*)&_m;
			T* v = (T*)&_v0;
			T* t = (T*)&_t;
			if (Size == 2)
			{
				t[0] = v[0] * m[0] + v[1] * m[2];
				t[1] = v[0] * m[1] + v[1] * m[3];
			}
			if (Size == 3)
			{
				t[0] = v[0] * m[0] + v[1] * m[3] + v[2] * m[6];
				t[1] = v[0] * m[1] + v[1] * m[4] + v[2] * m[7];
				t[2] = v[0] * m[2] + v[1] * m[5] + v[2] * m[8];
			}
			if (Size == 4)
			{
				t[0] = v[0] * m[0] + v[1] * m[4] + v[2] * m[8] + v[3] * m[12];
				t[1] = v[0] * m[1] + v[1] * m[5] + v[2] * m[9] + v[3] * m[13];
				t[2] = v[0] * m[2] + v[1] * m[6] + v[2] * m[10] + v[3] * m[14];
				t[3] = v[0] * m[3] + v[1] * m[7] + v[2] * m[11] + v[3] * m[15];
			}
			return _t;
		}


		friend TMatrix operator*(const TMatrix &m, const TMatrix &m1)
		{
			static_assert(Size >= 2 && Size <= 4, "only 2d,3d,4d matrix supported");
			TMatrix t;
			if (Size == 2)
			{
#define rc2(r, c) m.Get(0,r)*m1.Get(c,0)+m.Get(1,r)*m1.Get(c,1)
				t[0][0] = rc2(0, 0); t[1][0] = rc2(0, 1);
				t[0][1] = rc2(1, 0); t[1][1] = rc2(1, 1);
#undef rc2 
			}
			if (Size == 3)
			{
#define rc3(r, c) m.Get(0,r)*m1.Get(c,0)+m.Get(1,r)*m1.Get(c,1)+m.Get(2,r)*m1.Get(c,2)
				t[0][0] = rc3(0, 0); t[1][0] = rc3(0, 1); t[2][0] = rc3(0, 2);
				t[0][1] = rc3(1, 0); t[1][1] = rc3(1, 1); t[2][1] = rc3(1, 2);
				t[0][2] = rc3(2, 0); t[1][2] = rc3(2, 1); t[2][2] = rc3(2, 2);
#undef rc3 
			}
			if (Size == 4)
			{
#define rc4(r, c) m.Get(0,r)*m1.Get(c,0)+m.Get(1,r)*m1.Get(c,1)+m.Get(2,r)*m1.Get(c,2)+m.Get(3,r)*m1.Get(c,3)
				t[0][0] = rc4(0, 0); t[1][0] = rc4(0, 1); t[2][0] = rc4(0, 2);  t[3][0] = rc4(0, 3);
				t[0][1] = rc4(1, 0); t[1][1] = rc4(1, 1); t[2][1] = rc4(1, 2);  t[3][1] = rc4(1, 3);
				t[0][2] = rc4(2, 0); t[1][2] = rc4(2, 1); t[2][2] = rc4(2, 2);  t[3][2] = rc4(2, 3);
				t[0][3] = rc4(3, 0); t[1][3] = rc4(3, 1); t[2][3] = rc4(3, 2);  t[3][3] = rc4(3, 3);
#undef rc4 
			}
			return t;
		}

		void operator*=(const TMatrix &matrix)
		{
			*this = *this*matrix;
		}
		void operator=(const TMatrix& matrix)
		{
			for (int i = 0; i < Size; i++)
				m[i] = matrix.m[i];
		}
		T Get(int col, int row)const
		{
			return m[col][row];
		}
		TVec<T, Size> operator[](int id)const
		{
			assert(id >= 0 && id < Size);
			return m[id];
		}
		TVec<T, Size>& operator[](int id)
		{
			assert(id >= 0 && id < Size);
			return m[id];
		}
		void SetOffset(const TVec<T, Size - 1>& use_off)
		{
			*(TVec<T, Size - 1>*)(&m[high]) = use_off;
		}
		TVec<T, Size - 1> GetOffset()const
		{
			return *(TVec<T, Size - 1>*)(&m[high]);
		}
		void SetRotation(const TMatrix<T, Size - 1>& use_rot)
		{
			for (int i = 0; i < Size - 1; i++)
			{
				*(TVec<T, Size - 1>*)&m[i] = use_rot.m[i];
			}
		}
		TMatrix<T, Size - 1> GetRotation()const
		{
			TMatrix<T, Size - 1> t;
			for (int i = 0; i < Size - 1; i++)
			{
				*(TVec<T, Size - 1>*)&t[i] = m[i];
			}
			return t;
		}

		static TMatrix<T, 4> GetPerspective(T fov_deg, T width, T height, T zNear, T zFar)
		{
			T w = tan(fov_deg*(T)(M_PI / 360.0));
			T h = (w * height) / width;

			return TMatrix<T, 4>(
				1.0f / w, 0, 0, 0,
				0, 1.0f / h, 0, 0,
				0, 0, (zFar + zNear) / (zNear - zFar), (2 * zFar * zNear) / (zNear - zFar),
				0, 0, -1, 0);
		}
		static TMatrix<T, 4> GetOrtho(T left, T right, T bottom, T top, T zNear, T zFar)
		{
			T tx = -(right + left) / (right - left);
			T ty = -(top + bottom) / (top - bottom);
			T tz = -(zFar + zNear) / (zFar - zNear);
			return TMatrix<T, 4>(
				2 / (right - left), 0, 0, tx,
				0, 2 / (top - bottom), 0, ty,
				0, 0, -2 / (zFar - zNear), tz,
				0, 0, 0, 1);
		}
		static TMatrix<T, 4> GetOrtho(TVec<T, 2> pos, TVec<T, 2> size, T z_near, T z_far)
		{
			size *= 0.5;
			return GetOrtho(pos[0] - size[0], pos[0] + size[0], pos[1] - size[1], pos[1] + size[1], z_near, z_far);
		}
		static TMatrix<T, 4> GetMirror(const TPlane<T, 3>& plane)
		{
			T nx = plane.normal[0];
			T ny = plane.normal[1];
			T nz = plane.normal[2];
			T k = plane.dist;
			return TMatrix<T, 4>(
				1 - 2 * nx*nx, -2 * nx*ny, -2 * nx*nz, -2 * nx*k,
				-2 * nx*ny, 1 - 2 * ny*ny, -2 * ny*nz, -2 * ny*k,
				-2 * nx*nz, -2 * ny*nz, 1 - 2 * nz*nz, -2 * nz*k,
				0, 0, 0, 1);
		}
		static TMatrix<T, 4> GetCubeView(const int side)// 0-"+x" 1-"+y" 2-"+z" 3-"-x" 4-"-y" 5-"-z"
		{
			assert(side >= 0 && side <= 5);
			switch (side)
			{
			case 0:
				return TMatrix<T, 4>(
					0, 0, -1, 0,
					0, 1, 0, 0,
					1, 0, 0, 0,
					0, 0, 0, 1);

			case 1:
				return TMatrix<T, 4>(
					1, 0, 0, 0,
					0, 0, -1, 0,
					0, 1, 0, 0,
					0, 0, 0, 1);

			case 2:
				return TMatrix<T, 4>(
					1, 0, 0, 0,
					0, 1, 0, 0,
					0, 0, 1, 0,
					0, 0, 0, 1);
			case 3:
				return TMatrix<T, 4>(
					0, 0, 1, 0,
					0, 1, 0, 0,
					-1, 0, 0, 0,
					0, 0, 0, 1);
			case 4:
				return TMatrix<T, 4>(
					1, 0, 0, 0,
					0, 0, 1, 0,
					0, -1, 0, 0,
					0, 0, 0, 1);
			case 5:
				return TMatrix<T, 4>(
					-1, 0, 0, 0,
					0, 1, 0, 0,
					0, 0, -1, 0,
					0, 0, 0, 1);
			}
			return TMatrix<T, 4>();
		}
		static TMatrix<T, 4> GetCubeProjection(const T zNear, const T zFar)
		{
			return TMatrix<T, 4>(
				1, 0, 0, 0,
				0, -1, 0, 0,
				0, 0, (zFar + zNear) / (zFar - zNear), -(2 * zFar * zNear) / (zFar - zNear),
				0, 0, 1, 0);
		}

		T GetDet()const
		{
			static_assert(Size >= 2 && Size <= 4, "only 2d,3d,4d matrix supported");
			if (Size == 2)
			{
				return m[0][0] * m[1][1] - m[0][1] * m[1][0];
			}
			if (Size == 3)
			{
				return
					m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
					m[1][0] * (m[0][1] * m[2][2] - m[0][2] * m[2][1]) +
					m[2][0] * (m[0][1] * m[1][2] - m[0][2] * m[1][1]);
			}
			if (Size == 4)
			{
				return
					m[0][0] * (
					m[1][1] * (m[2][2] * m[3][3] - m[2][3] * m[3][2]) -
					m[2][1] * (m[1][2] * m[3][3] - m[1][3] * m[3][2]) +
					m[3][1] * (m[1][2] * m[2][3] - m[1][3] * m[2][2])
					) -
					m[1][0] * (
					m[0][1] * (m[2][2] * m[3][3] - m[2][3] * m[3][2]) -
					m[2][1] * (m[0][2] * m[3][3] - m[0][3] * m[3][2]) +
					m[3][1] * (m[0][2] * m[2][3] - m[0][3] * m[2][2])
					) +
					m[2][0] * (
					m[0][1] * (m[1][2] * m[3][3] - m[1][3] * m[3][2]) -
					m[1][1] * (m[0][2] * m[3][3] - m[0][3] * m[3][2]) +
					m[3][1] * (m[0][2] * m[1][3] - m[0][3] * m[1][2])
					) -
					m[3][0] * (
					m[0][1] * (m[1][2] * m[2][3] - m[1][3] * m[2][2]) -
					m[1][1] * (m[0][2] * m[2][3] - m[0][3] * m[2][2]) +
					m[2][1] * (m[0][2] * m[1][3] - m[0][3] * m[1][2])
					);
			}
		}
		TMatrix GetInverted()const
		{
			static_assert(Size >= 2 && Size <= 4, "only 2d,3d,4d matrix supported");
			TMatrix mat;

			if (Size == 2)
			{
				mat[0][0] = m[1][1]; mat[1][0] = -m[0][1];
				mat[0][1] = -m[1][0]; mat[1][1] = m[0][0];
				return mat*((T)(1) / GetDet());
			}
			if (Size == 3)
			{
				mat[0][0] = m[1][1] * m[2][2] - m[1][2] * m[2][1];
				mat[0][1] = m[0][2] * m[2][1] - m[0][1] * m[2][2];
				mat[0][2] = m[0][1] * m[1][2] - m[0][2] * m[1][1];

				mat[1][0] = m[1][2] * m[2][0] - m[1][0] * m[2][2];
				mat[1][1] = m[0][0] * m[2][2] - m[0][2] * m[2][0];
				mat[1][2] = m[0][2] * m[1][0] - m[0][0] * m[1][2];

				mat[2][0] = m[1][0] * m[2][1] - m[1][1] * m[2][0];
				mat[2][1] = m[0][1] * m[2][0] - m[0][0] * m[2][1];
				mat[2][2] = m[0][0] * m[1][1] - m[0][1] * m[1][0];
				return mat*((T)(1) / (mat[0][0] * m[0][0] + mat[0][1] * m[1][0] + mat[0][2] * m[2][0]));
			}
			if (Size == 4)
			{
#define get(r,c) m[c][r]

				float p00 = get(2, 2) * get(3, 3);
				float p01 = get(3, 2) * get(2, 3);
				float p02 = get(1, 2) * get(3, 3);
				float p03 = get(3, 2) * get(1, 3);
				float p04 = get(1, 2) * get(2, 3);
				float p05 = get(2, 2) * get(1, 3);
				float p06 = get(0, 2) * get(3, 3);
				float p07 = get(3, 2) * get(0, 3);
				float p08 = get(0, 2) * get(2, 3);
				float p09 = get(2, 2) * get(0, 3);
				float p10 = get(0, 2) * get(1, 3);
				float p11 = get(1, 2) * get(0, 3);

				mat[0][0] = (p00 * get(1, 1) + p03 * get(2, 1) + p04 * get(3, 1)) - (p01 * get(1, 1) + p02 * get(2, 1) + p05 * get(3, 1));
				mat[1][0] = (p01 * get(0, 1) + p06 * get(2, 1) + p09 * get(3, 1)) - (p00 * get(0, 1) + p07 * get(2, 1) + p08 * get(3, 1));
				mat[2][0] = (p02 * get(0, 1) + p07 * get(1, 1) + p10 * get(3, 1)) - (p03 * get(0, 1) + p06 * get(1, 1) + p11 * get(3, 1));
				mat[3][0] = (p05 * get(0, 1) + p08 * get(1, 1) + p11 * get(2, 1)) - (p04 * get(0, 1) + p09 * get(1, 1) + p10 * get(2, 1));
				mat[0][1] = (p01 * get(1, 0) + p02 * get(2, 0) + p05 * get(3, 0)) - (p00 * get(1, 0) + p03 * get(2, 0) + p04 * get(3, 0));
				mat[1][1] = (p00 * get(0, 0) + p07 * get(2, 0) + p08 * get(3, 0)) - (p01 * get(0, 0) + p06 * get(2, 0) + p09 * get(3, 0));
				mat[2][1] = (p03 * get(0, 0) + p06 * get(1, 0) + p11 * get(3, 0)) - (p02 * get(0, 0) + p07 * get(1, 0) + p10 * get(3, 0));
				mat[3][1] = (p04 * get(0, 0) + p09 * get(1, 0) + p10 * get(2, 0)) - (p05 * get(0, 0) + p08 * get(1, 0) + p11 * get(2, 0));

				float q00 = get(2, 0) * get(3, 1);
				float q01 = get(3, 0) * get(2, 1);
				float q02 = get(1, 0) * get(3, 1);
				float q03 = get(3, 0) * get(1, 1);
				float q04 = get(1, 0) * get(2, 1);
				float q05 = get(2, 0) * get(1, 1);
				float q06 = get(0, 0) * get(3, 1);
				float q07 = get(3, 0) * get(0, 1);
				float q08 = get(0, 0) * get(2, 1);
				float q09 = get(2, 0) * get(0, 1);
				float q10 = get(0, 0) * get(1, 1);
				float q11 = get(1, 0) * get(0, 1);

				mat[0][2] = (q00 * get(1, 3) + q03 * get(2, 3) + q04 * get(3, 3)) - (q01 * get(1, 3) + q02 * get(2, 3) + q05 * get(3, 3));
				mat[1][2] = (q01 * get(0, 3) + q06 * get(2, 3) + q09 * get(3, 3)) - (q00 * get(0, 3) + q07 * get(2, 3) + q08 * get(3, 3));
				mat[2][2] = (q02 * get(0, 3) + q07 * get(1, 3) + q10 * get(3, 3)) - (q03 * get(0, 3) + q06 * get(1, 3) + q11 * get(3, 3));
				mat[3][2] = (q05 * get(0, 3) + q08 * get(1, 3) + q11 * get(2, 3)) - (q04 * get(0, 3) + q09 * get(1, 3) + q10 * get(2, 3));
				mat[0][3] = (q02 * get(2, 2) + q05 * get(3, 2) + q01 * get(1, 2)) - (q04 * get(3, 2) + q00 * get(1, 2) + q03 * get(2, 2));
				mat[1][3] = (q08 * get(3, 2) + q00 * get(0, 2) + q07 * get(2, 2)) - (q06 * get(2, 2) + q09 * get(3, 2) + q01 * get(0, 2));
				mat[2][3] = (q06 * get(1, 2) + q11 * get(3, 2) + q03 * get(0, 2)) - (q10 * get(3, 2) + q02 * get(0, 2) + q07 * get(1, 2));
				mat[3][3] = (q10 * get(2, 2) + q04 * get(0, 2) + q09 * get(1, 2)) - (q08 * get(1, 2) + q11 * get(2, 2) + q05 * get(0, 2));
#undef get
				return mat * (1 / (m[0][0] * mat[0][0] + m[0][1] * mat[1][0] + m[0][2] * mat[2][0] + m[0][3] * mat[3][0]));
			}
		}
		void Invert()
		{
			*this = GetInverted();
		}
	};


	typedef TMatrix<float, 2> TMatrix2;
	typedef TMatrix<double, 2> TMatrix2d;
	typedef TMatrix<float, 3> TMatrix3;
	typedef TMatrix<double, 2> TMatrix3d;
	typedef TMatrix<float, 4> TMatrix4;
	typedef TMatrix<double, 4> TMatrix4d;
}