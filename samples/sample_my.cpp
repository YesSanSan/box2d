#include "car.h"
#include "donut.h"
#include "draw.h"
#include "human.h"
#include "sample.h"

#include "box2d/box2d.h"
#include "box2d/math_functions.h"
#include "box2d/types.h"

#include <GLFW/glfw3.h>
#include <algorithm>
#include <cmath>
#include <imgui.h>
#include <numbers>

class SampleMyTest : public Sample
{
	b2BodyId m_chassisId;
	b2BodyId m_wheelId;
	b2JointId m_axleId;

	struct PID
	{
		float kp;
		float ki;
		float kd;
		float i_limit;
		float out_limit;
		struct info
		{
			float last;
			float sum;
			bool init;
		} info;
	};

	PID pid_angle;
	PID pid_speed;

	float target_angle;
	float target_speed;
	float target_pos;

	bool use_speed;

public:
	explicit SampleMyTest( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 8.0f, 25.0f };
			m_context->camera.zoom = 60.0f;
		}

		{
			float gridSize = 1.0f;

			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();

			b2Polygon box = b2MakeOffsetBox( 50, 1, b2Vec2{ 0, 0 }, b2Rot_identity );
			b2CreatePolygonShape( groundId, &shapeDef, &box );
			box = b2MakeOffsetBox( 50, 1, b2Vec2{ 0, 100 }, b2Rot_identity );
			b2CreatePolygonShape( groundId, &shapeDef, &box );
			box = b2MakeOffsetBox( 1, 50, b2Vec2{ -50, 50 }, b2Rot_identity );
			b2CreatePolygonShape( groundId, &shapeDef, &box );
			box = b2MakeOffsetBox( 1, 50, b2Vec2{ 50, 50 }, b2Rot_identity );
			b2CreatePolygonShape( groundId, &shapeDef, &box );
		}

		m_chassisId = {};
		m_wheelId = {};
		m_axleId = {};
		pid_angle = { .kp = 2000, .ki = 40, .kd = 8000, .i_limit = 20, .out_limit = 500 };
		pid_speed = { .kp = 0.1, .ki = 0.1, .kd = 0.1, .i_limit = 0.2, .out_limit = 0.3 };
		target_angle = 0;
		target_speed = 0;
		target_pos = 0;
		use_speed = 0;

		{

			b2Vec2 position = { 0.f, 3.f };
			float scale = 2;
			float hertz = 10;
			float dampingRatio = 0.7;
			float torque = 0;

			assert( B2_IS_NULL( m_chassisId ) );
			assert( B2_IS_NULL( m_wheelId ) );

			b2Vec2 vertices[6] = {
				{ -0.8f, -0.5f }, { 0.8f, -0.5f }, { 1.5f, 0.0f }, { 0.0f, 0.9f }, { -1.15f, 0.9f }, { -1.5f, 0.2f },
			};

			for ( int i = 0; i < 6; ++i )
			{
				vertices[i].x *= 0.85f * scale;
				vertices[i].y *= 0.85f * scale;
			}

			b2Hull hull = b2ComputeHull( vertices, 6 );
			b2Polygon chassis = b2MakePolygon( &hull, 0.15f * scale );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.density = 1.0f / scale;
			shapeDef.material.friction = 0.2f;

			b2Circle circle = { { 0.0f, 0.0f }, 0.6f * scale };

			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = b2Add( { 0.0f, 1.0f * scale }, position );
			bodyDef.enableSleep = false;
			m_chassisId = b2CreateBody( m_worldId, &bodyDef );
			b2CreatePolygonShape( m_chassisId, &shapeDef, &chassis );

			shapeDef.density = 0.3f / scale;
			shapeDef.material.friction = 2.f;
			shapeDef.material.rollingResistance = 0.1f;

			bodyDef.position = b2Add( { 0, 0.15f * scale }, position );
			bodyDef.type = b2_dynamicBody;
			bodyDef.allowFastRotation = true;
			bodyDef.enableSleep = false;
			bodyDef.angularDamping = 2;
			m_wheelId = b2CreateBody( m_worldId, &bodyDef );
			b2CreateCircleShape( m_wheelId, &shapeDef, &circle );

			b2Vec2 axis = { 0.0f, 1.0f };
			b2Vec2 pivot = b2Body_GetPosition( m_wheelId );

			// float throttle = 0.0f;
			// float speed = 35.0f;
			// float torque = 2.5f * scale;
			// float hertz = 5.0f;
			// float dampingRatio = 0.7f;

			b2WheelJointDef jointDef = b2DefaultWheelJointDef();

			jointDef.base.bodyIdA = m_chassisId;
			jointDef.base.bodyIdB = m_wheelId;
			jointDef.base.localFrameA.q = b2MakeRot( 0.5f * B2_PI );
			jointDef.base.localFrameA.p = b2Body_GetLocalPoint( jointDef.base.bodyIdA, pivot );
			jointDef.base.localFrameB.p = b2Body_GetLocalPoint( jointDef.base.bodyIdB, pivot );
			jointDef.motorSpeed = 0.0f;
			jointDef.maxMotorTorque = torque;
			jointDef.enableMotor = false;
			jointDef.hertz = hertz;
			jointDef.dampingRatio = dampingRatio;
			jointDef.lowerTranslation = -0.25f * scale;
			jointDef.upperTranslation = 0.25f * scale;
			jointDef.enableLimit = true;
			m_axleId = b2CreateWheelJoint( m_worldId, &jointDef );
		}
	}

	float pid_update( PID& pid, float cur, float target )
	{
		if ( !pid.info.init ) [[unlikely]]
		{
			pid.info.init = true;
			pid.info.last = cur;
		}
		float err = target - cur;
		float out = err * pid.kp + pid.info.sum - ( cur - pid.info.last ) * pid.kd;
		pid.info.last = cur;
		pid.info.sum = std::clamp( pid.info.sum + err * pid.ki, -pid.i_limit, pid.i_limit );
		out = std::clamp( out, -pid.out_limit, pid.out_limit );
		return out;
	}

	void Step() override
	{
		if ( m_context->pause == false || m_context->singleStep == true )
		{
			auto rot = b2Body_GetRotation( m_chassisId );
			auto angle = b2Rot_GetAngle( rot );
			auto pos = b2Body_GetPosition( m_chassisId );
			auto t = b2WheelJoint_GetMotorTorque( m_axleId );
			DrawScreenTextLine( "rot: %g", rot );
			DrawScreenTextLine( "angle: %g", angle );
			DrawScreenTextLine( "pos: %g, %g", pos.x, pos.y );
			DrawScreenTextLine( "t: %g", t );

			auto m1 = b2Body_GetWorldCenterOfMass( m_chassisId );
			auto m2 = b2Body_GetWorldCenterOfMass( m_wheelId );
			auto v = m1 - m2;
			float angle_m = atan2( v.y, v.x ) - std::numbers::pi / 2;

			auto cur_speed = b2Body_GetLinearVelocity( m_chassisId ).x;
			float spd_out = pid_update( pid_speed, cur_speed, target_speed );
			if ( use_speed )
				target_angle = spd_out;

			float out_t = pid_update( pid_angle, -angle, target_angle );
			float out_ = std::clamp( out_t + 1020 * std::sin( angle_m ), -pid_angle.out_limit, pid_angle.out_limit );

			b2Body_ApplyTorque( m_chassisId, -out_, 1 );
			b2Body_ApplyTorque( m_wheelId, out_, 1 );
			// b2Body_ApplyTorque( m_chassisId, -pid_angle.kp * sin( angle_m ), 1 );
			// b2Body_ApplyTorque( m_wheelId, pid_angle.kp * sin( angle_m ), 1 );
			// b2WheelJoint_SetMotorSpeed( m_axleId, speed );
			// b2Joint_WakeBodies( m_axleId );
			DrawScreenTextLine( "angle_m: %g", angle_m );
			DrawScreenTextLine( "pid_t: %g", out_t );
			DrawScreenTextLine( "out_t: %g", out_ );
		}

		Sample::Step();
	}

	bool DrawControls() override
	{
		ImGui::PushItemWidth( 10.0f * ImGui::GetFontSize() );
		// ImGui::SliderFloat( "kp", &pid_angle.kp, 0.0f, 10000.0f, "%.0f" );
		// ImGui::SliderFloat( "ki", &pid_angle.ki, 0.0f, 100.0f, "%.2f" );
		// ImGui::SliderFloat( "i_lmt", &pid_angle.i_limit, 0.0f, 100.0f, "%.2f" );
		// ImGui::SliderFloat( "kd", &pid_angle.kd, 0.0f, 10000.0f, "%.0f" );

		ImGui::SliderFloat( "kp", &pid_speed.kp, 0.0f, 1.0f, "%.3f" );
		ImGui::SliderFloat( "ki", &pid_speed.ki, 0.0f, 1.0f, "%.3f" );
		ImGui::SliderFloat( "i_lmt", &pid_speed.i_limit, 0.0f, 1.0f, "%.3f" );
		ImGui::SliderFloat( "kd", &pid_speed.kd, 0.0f, 10.0f, "%.2f" );

		ImGui::SliderFloat( "tgt_a", &target_angle, -0.65f, 0.7f, "%.2f" );
		ImGui::SliderFloat( "tgt_v", &target_speed, -1.0f, 1.0f, "%.2f" );
		ImGui::PopItemWidth();
		// if ( ImGui::Button( "stop" ) )
		// {
		// 	m_speed = 0.0f;
		// }

		ImGui::Checkbox( "use speed", &use_speed );

		// ImGui::Text( "world size = %g kilometers", m_gridSize * m_gridCount / 1000.0f );

		return true;
	}

	static Sample* Create( SampleContext* context )
	{
		return new SampleMyTest( context );
	}
};

static int SampleMyTest = RegisterSample( "my", "test", SampleMyTest::Create );
