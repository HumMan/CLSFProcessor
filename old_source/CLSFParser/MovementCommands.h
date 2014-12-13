#pragma once

struct TLinearCommand:public TMovementCommand
{
	TVec3d pos;
	bool has_dir;
	TVec3d dir;
	bool has_contact_point;
	TVec3d contact_point;
	TLinearCommand()
		:has_dir(false)
		,has_contact_point(false)
	{}
	void SetDir(const TVec3d& use_dir);
	void SetEndPos(const TVec3d& use_pos);
	void SetContact(const TVec3d& use_contact);
	TMovementCommand* do_clone()const;
	void GetGCode(std::string& program)
	{
		char buf[1000];
		sprintf(buf,"G01 X %lf Y %lf Z %lf\n",pos[0],pos[1],pos[2]);
		program+=buf;
	}
};

struct TCircleCommand:public TMovementCommand
{
	TVec3d pos;
	TVec3d 
		center,
		normal;
	std::vector<double> circle_params;
	bool has_spiral_times;
	int spiral_times;
	TCircleCommand()
		:has_spiral_times(false)
	{}
	void SetDir(const TVec3d& use_dir);
	void SetEndPos(const TVec3d& use_pos);
	void SetContact(const TVec3d& use_contact);
	TMovementCommand* do_clone()const;
	void GetGCode(std::string& program)
	{
		char buf[1000];
		sprintf(buf,"G02 X %lf Y %lf Z %lf\n",pos[0],pos[1],pos[2]);
		program+=buf;
	}
};

struct TFromCommand:public TMovementCommand
{
	TVec3d pos;
	bool has_dir;
	TVec3d dir;
	TFromCommand()
		:has_dir(false)
	{}
	void SetDir(const TVec3d& use_dir);
	void SetEndPos(const TVec3d& use_pos);
	void SetContact(const TVec3d& use_contact);
	TMovementCommand* do_clone()const;
	void GetGCode(std::string& program)
	{
		char buf[1000];
		sprintf(buf,"FROM X %lf Y %lf Z %lf\n",pos[0],pos[1],pos[2]);
		program+=buf;
	}
};

struct TGoHomeCommand:public TMovementCommand
{
	TVec3d pos;
	bool has_dir;
	TVec3d dir;
	TGoHomeCommand()
		:has_dir(false)
	{}
	void SetDir(const TVec3d& use_dir);
	void SetEndPos(const TVec3d& use_pos);
	void SetContact(const TVec3d& use_contact);
	TMovementCommand* do_clone()const;
	void GetGCode(std::string& program)
	{
		char buf[1000];
		sprintf(buf,"GOHOME X %lf Y %lf Z %lf\n",pos[0],pos[1],pos[2]);
		program+=buf;
	}
};

void TLinearCommand::SetDir(const TVec3d& use_dir)
{
	has_dir=true;
	dir=use_dir;
}

void TLinearCommand::SetEndPos(const TVec3d& use_pos)
{
	pos=use_pos;
}

void TLinearCommand::SetContact(const TVec3d& use_contact)
{
	has_contact_point=true;
	contact_point=use_contact;
}

TMovementCommand* TLinearCommand::do_clone()const
{
	return new TLinearCommand( *this );
}

void TCircleCommand::SetDir(const TVec3d& use_dir)
{
	throw std::string("Error: CIRCLE has no dir!");
}

void TCircleCommand::SetEndPos(const TVec3d& use_pos)
{
	pos=use_pos;
}

void TCircleCommand::SetContact(const TVec3d& use_contact)
{
	throw std::string("Error: CIRCLE has no contact!");
}

TMovementCommand* TCircleCommand::do_clone()const
{
	return new TCircleCommand( *this );
}

void TFromCommand::SetDir(const TVec3d& use_dir)
{
	has_dir=true;
	dir=use_dir;
}

void TFromCommand::SetEndPos(const TVec3d& use_pos)
{
	pos=use_pos;
}

void TFromCommand::SetContact(const TVec3d& use_contact)
{
	throw std::string("Error: FROM has no contact!");
}

TMovementCommand* TFromCommand::do_clone()const
{
	return new TFromCommand( *this );
}

void TGoHomeCommand::SetDir(const TVec3d& use_dir)
{
	has_dir=true;
	dir=use_dir;
}

void TGoHomeCommand::SetEndPos(const TVec3d& use_pos)
{
	pos=use_pos;
}

void TGoHomeCommand::SetContact(const TVec3d& use_contact)
{
	throw std::string("Error: GOHOME has no contact!");
}

TMovementCommand* TGoHomeCommand::do_clone()const
{
	return new TGoHomeCommand( *this );
}