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
#include <fstream>
#include <iomanip>
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

	float manual_target_angle;
	float speed_target_angle;
	float command_angle;
	float target_speed;
	float target_pos;
	float filtered_speed;
	float speed_filter_tau;
	float speed_angle_rate_limit;

	bool use_speed;
	bool speed_filter_init;
	bool speed_log_active;
	float speed_log_time;
	std::ofstream speed_log;

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
		pid_speed = { .kp = 0.08f, .ki = 0.125f, .kd = 0.0f, .i_limit = 0.225f, .out_limit = 0.25f };
		manual_target_angle = 0;
		speed_target_angle = 0;
		command_angle = 0;
		target_speed = 0;
		target_pos = 0;
		filtered_speed = 0;
		speed_filter_tau = 0.12f;
		speed_angle_rate_limit = 0.8f;
		use_speed = 1;
		speed_filter_init = false;
		speed_log_active = false;
		speed_log_time = 0.0f;

		{

			b2Vec2 position = { 0.f, 3.f };
			float scale = 2;
			float hertz = 15;
			float dampingRatio = 0.8;
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

	~SampleMyTest() override
	{
		end_speed_log();
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

	void reset_pid( PID& pid )
	{
		pid.info.last = 0.0f;
		pid.info.sum = 0.0f;
		pid.info.init = false;
	}

	float speed_pid_update( PID& pid, float cur, float target, float dt )
	{
		if ( dt <= 0.0f )
		{
			return std::clamp( pid.info.sum, -pid.out_limit, pid.out_limit );
		}

		if ( !pid.info.init ) [[unlikely]]
		{
			pid.info.init = true;
			pid.info.last = cur;
		}

		float err = target - cur;
		float derivative = -( cur - pid.info.last ) / dt;
		float unclamped = err * pid.kp + pid.info.sum + derivative * pid.kd;
		float out = std::clamp( unclamped, -pid.out_limit, pid.out_limit );

		bool saturatedHigh = unclamped > pid.out_limit && err > 0.0f;
		bool saturatedLow = unclamped < -pid.out_limit && err < 0.0f;
		if ( saturatedHigh == false && saturatedLow == false )
		{
			pid.info.sum = std::clamp( pid.info.sum + err * pid.ki * dt, -pid.i_limit, pid.i_limit );
			unclamped = err * pid.kp + pid.info.sum + derivative * pid.kd;
			out = std::clamp( unclamped, -pid.out_limit, pid.out_limit );
		}

		pid.info.last = cur;
		return out;
	}

	float filter_speed( float cur_speed, float dt )
	{
		if ( speed_filter_init == false || speed_filter_tau <= 0.0f || dt <= 0.0f )
		{
			filtered_speed = cur_speed;
			speed_filter_init = true;
			return filtered_speed;
		}

		float alpha = dt / ( speed_filter_tau + dt );
		filtered_speed += alpha * ( cur_speed - filtered_speed );
		return filtered_speed;
	}

	float slew_angle( float current, float target, float dt ) const
	{
		if ( dt <= 0.0f || speed_angle_rate_limit <= 0.0f )
		{
			return target;
		}

		float maxDelta = speed_angle_rate_limit * dt;
		return current + std::clamp( target - current, -maxDelta, maxDelta );
	}

	void begin_speed_log()
	{
		speed_log.close();
		speed_log.open( "sample_my_speed.csv", std::ios::out | std::ios::trunc );
		if ( speed_log.is_open() )
		{
			speed_log << "time,target_speed,actual_speed,filtered_speed,speed_angle,command_angle\n";
			speed_log << std::fixed << std::setprecision( 6 );
		}

		speed_log_time = 0.0f;
		speed_log_active = true;
	}

	void end_speed_log()
	{
		if ( speed_log.is_open() )
		{
			speed_log.flush();
			speed_log.close();
		}

		speed_log_active = false;
		speed_log_time = 0.0f;
	}

	void write_speed_log( float dt, float actual_speed )
	{
		if ( speed_log.is_open() == false )
		{
			return;
		}

		speed_log << speed_log_time << ',' << target_speed << ',' << actual_speed << ',' << filtered_speed << ',' << speed_target_angle
				  << ',' << command_angle << '\n';
		speed_log_time += dt;
	}

	void Step() override
	{
		if ( m_context->pause == false || m_context->singleStep == true )
		{
			float dt = m_context->hertz > 0.0f ? 1.0f / m_context->hertz : 0.0f;
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
			filter_speed( cur_speed, dt );
			if ( use_speed )
			{
				if ( speed_log_active == false )
				{
					begin_speed_log();
				}

				speed_target_angle = speed_pid_update( pid_speed, filtered_speed, target_speed, dt );
				command_angle = slew_angle( command_angle, speed_target_angle, dt );
				write_speed_log( dt, cur_speed );
			}
			else
			{
				if ( speed_log_active )
				{
					end_speed_log();
				}

				reset_pid( pid_speed );
				speed_filter_init = false;
				speed_target_angle = 0.0f;
				command_angle = manual_target_angle;
			}

			float out_t = pid_update( pid_angle, -angle, command_angle );
			float out_ = std::clamp( out_t + 1020 * std::sin( angle_m ), -pid_angle.out_limit, pid_angle.out_limit );

			b2Body_ApplyTorque( m_chassisId, -out_, 1 );
			b2Body_ApplyTorque( m_wheelId, out_, 1 );
			// b2Body_ApplyTorque( m_chassisId, -pid_angle.kp * sin( angle_m ), 1 );
			// b2Body_ApplyTorque( m_wheelId, pid_angle.kp * sin( angle_m ), 1 );
			// b2WheelJoint_SetMotorSpeed( m_axleId, speed );
			// b2Joint_WakeBodies( m_axleId );
			DrawScreenTextLine( "angle_m: %g", angle_m );
			DrawScreenTextLine( "filtered_speed: %g", filtered_speed );
			DrawScreenTextLine( "speed_angle: %g", speed_target_angle );
			DrawScreenTextLine( "command_angle: %g", command_angle );
			DrawScreenTextLine( "pid_t: %g", out_t );
			DrawScreenTextLine( "out_t: %g", out_ );
			DrawScreenTextLine( "speed csv: %s", speed_log.is_open() ? "sample_my_speed.csv" : "off" );
		}

		Sample::Step();
	}

	bool DrawControls() override
	{
		ImGui::PushItemWidth( 6.0f * ImGui::GetFontSize() );
		// ImGui::SliderFloat( "kp", &pid_angle.kp, 0.0f, 10000.0f, "%.0f" );
		// ImGui::SliderFloat( "ki", &pid_angle.ki, 0.0f, 100.0f, "%.2f" );
		// ImGui::SliderFloat( "i_lmt", &pid_angle.i_limit, 0.0f, 100.0f, "%.2f" );
		// ImGui::SliderFloat( "kd", &pid_angle.kd, 0.0f, 10000.0f, "%.0f" );

		ImGui::SliderFloat( "speed kp", &pid_speed.kp, 0.0f, 1.0f, "%.3f" );
		ImGui::SliderFloat( "speed ki", &pid_speed.ki, 0.0f, 1.0f, "%.3f" );
		ImGui::SliderFloat( "speed i_lmt", &pid_speed.i_limit, 0.0f, 1.0f, "%.3f" );
		ImGui::SliderFloat( "speed angle_lmt", &pid_speed.out_limit, 0.0f, 0.7f, "%.3f" );
		ImGui::SliderFloat( "speed rate_lmt", &speed_angle_rate_limit, 0.0f, 3.0f, "%.2f" );
		ImGui::SliderFloat( "speed filter", &speed_filter_tau, 0.0f, 1.0f, "%.3f" );

		ImGui::SliderFloat( "tgt_a", &manual_target_angle, -0.6f, 0.7f, "%.2f" );

		ImGui::SliderFloat( "tgt_v", &target_speed, -5.0f, 5.0f, "%.2f" );
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

static int SampleMyTest = RegisterSample( "my", "pid", SampleMyTest::Create );
