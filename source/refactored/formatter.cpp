//void GetGCode(std::vector<TToolMovementElement> &pipe,std::string& result_code)
//	{
//		result_code="";
//		if(pipe.size()==0)return;
//		char* buff=new char[10000000];//TODO сюда записывается текст из auxfun котоорый может быть очень болььшим
//		double curr_feed=0/*pipe[0].feed*/;
//		int curr_cutcom=0;
//		string curr_path_name;
//		double curr_spndl_rpm=pipe[0].spndl_rpm;
//		bool curr_clw=pipe[0].clw;
//		//if(curr_feed==0)curr_feed=1500;//TODO
//		int frame_number=1;
//
//		bool oriented_from_goto_engage_done=false;
//		
//		double curr_coord[5];//текущие координаты
//		double curr_coord_machine[5];//текущие координаты с учетом округления(используется при инкрементном режиме для устранения накапливающейся ошбки округления)
//		bool need_print[5]={0,0,0,0,0};
//
//		double* new_coord=&pipe[0].pos[0];
//		for(int c=0;c<5;c++)
//		{
//			curr_coord[c]=new_coord[c]+2*gcode_axis_prop[c].repeat_tol;
//			curr_coord_machine[c]=
//				gcode_axis_prop[c].rad_to_deg
//				?RadToDeg(new_coord[c])
//				:new_coord[c];
//		}
//
//		for(int i=0;i<pipe.size();i++)
//		{
//			new_coord=&pipe[i].pos[0];
//			//switch(pipe[i].mask)
//			{
//			//case PrimitiveMask::LINE:
//			//	{
//				bool is_path_begin=(curr_path_name!=pipe[i].path_name&&pipe[i].path_name!="");
//				if(is_path_begin)
//				{
//					oriented_from_goto_engage_done=false;
//					oriented_from_goto_engage_done=false;
//				}
//
//				curr_path_name=pipe[i].path_name;
//
//				int temp_id=-1;
//				for(int t=0;t<oriented_from_goto.size();t++)
//				{
//					if(oriented_from_goto[t].path_name==(curr_path_name))
//					{
//						temp_id=t;
//						break;
//					}
//				}
//				
//				bool is_oriented_from_goto_engage_start=temp_id!=-1&&!oriented_from_goto_engage_done&&(((i==0||is_path_begin)&&pipe[i].color==186)||(i!=0&&pipe[i-1].color!=186&&pipe[i].color==186));
//				bool is_oriented_from_goto_engage_end=temp_id!=-1&&!oriented_from_goto_engage_done&&(pipe[i].color==186&&pipe[i+1].color!=186)&&curr_path_name==pipe[i+1].path_name;
//
//				bool is_oriented_from_goto_retract_start=
//					temp_id!=-1&&
//					oriented_from_goto_engage_done&&
//					((i==0&&pipe[i].color==186)||(i!=0&&pipe[i-1].color!=186&&pipe[i].color==186));
//
//				bool is_oriented_from_goto_retract_end=
//					temp_id!=-1&&
//					oriented_from_goto_engage_done&&
//					((i!=pipe.size()&&pipe[i].color==186&&pipe[i+1].color!=186)||(i==pipe.size()&&pipe[i].color==186)||curr_path_name!=pipe[i+1].path_name);
//
//				if(is_oriented_from_goto_engage_start)
//				{
//					sprintf(buff,"\nM5\nSPOS=%f;engage_start\n",oriented_from_goto[temp_id].orientation);
//					result_code+=buff;
//				}
//
//				if(is_oriented_from_goto_retract_start)
//				{
//					sprintf(buff,"\nM5\nSPOS=%f;retract_start\n",oriented_from_goto[temp_id].orientation);
//					result_code+=buff;
//				}
//
//				//if(oriented_from_goto&&!oriented_from_goto_engage_done)
//				//{
//				//	if((i==0&&pipe[i].color==186)||(pipe[i-1].color!=186&&pipe[i].color==186))//спец комманда для подхода и отхода
//				//	{
//				//		sprintf(buff,"\nSPOS=%f\n",oriented_from_goto_orientation);
//				//		result_code+=buff;
//				//		oriented_from_goto_engage_done=true;
//				//	}
//				//}
//
//					if(!pipe[i].rapid&&pipe[i].feed!=curr_feed)
//					{
//						//sprintf(buff,"N%i G01 F%i\n",frame_number+=10,(int)pipe[i].feed);
//						//result_code+=buff;
//					}
//
//					if((1/*coord_repeat_tol*/<abs(pipe[i].spndl_rpm-curr_spndl_rpm)||curr_clw!=pipe[i].clw||i==0)
//						&&temp_id==-1)//если is_oriented_from_goto_engage то задается в подходе отходе
//					{
//						if(abs(curr_spndl_rpm-0)<0.001||i==0)
//							sprintf(buff,"M3S%i\n",/*pipe[i].clw?3:4,*/int(pipe[i].spndl_rpm));
//						else if(abs(pipe[i].spndl_rpm-0)<0.001)
//							sprintf(buff,"M5\n");
//						else sprintf(buff,"");
//						result_code+=buff;
//						curr_spndl_rpm=pipe[i].spndl_rpm;
//						curr_clw=pipe[i].clw;
//					}
//
//					bool force_rapid_change=false;
//					for(int c=0;c<5;c++)
//					{
//						bool last_need_print=need_print[c];
//						double tol=gcode_axis_prop[c].rad_to_deg?DegToRad(gcode_axis_prop[c].repeat_tol):gcode_axis_prop[c].repeat_tol;
//						need_print[c]=
//							tol<=abs(new_coord[c]-curr_coord[c])||!gcode_axis_prop[c].remove_repeat;
//						if(need_print[c]&&gcode_axis_prop[c].force_rapid_change)
//							force_rapid_change=true;
//						//если имеется функция разблокировки/блокировки оси то включаем ее
//						if(!last_need_print&&need_print[c]&&gcode_axis_prop[c].lock_header!="")
//						{
//							result_code+=gcode_axis_prop[c].lock_header;
//						}
//						if(last_need_print&&!need_print[c]&&gcode_axis_prop[c].lock_header!="")
//						{
//							result_code+=gcode_axis_prop[c].lock_footer;
//						}
//					}
//
//					sprintf(buff,"%s",pipe[i].auxfun.c_str());
//					result_code+=buff;
//					//если направление инструмента 0,0,-1 то надо сменить коррекцию на противоположную
//					if(pipe[i].tool_orient.dir.Distance(TVec<double,3>(0,0,-1))<0.0001)
//					{
//						if(pipe[i].cutcom==1)pipe[i].cutcom=2;
//						if(pipe[i].cutcom==2)pipe[i].cutcom=1;
//					}
//					if(pipe[i].cutcom!=curr_cutcom)
//					{	
//						sprintf(buff,"G%i",40+pipe[i].cutcom);
//						result_code+=buff;
//						curr_cutcom=pipe[i].cutcom;
//					}
//					//плоскость круговой/спиральной интерполяции
//					if(pipe[i].mask==PrimitiveMask::CIRCLE)
//					{
//						int rot_axis;
//						bool positive;
//						IsOrthogonalVector(pipe[i].normal,rot_axis,positive);
//						int rot_axis_to_plane[3]={19,18,17};
//						sprintf(buff,"G%i",rot_axis_to_plane[rot_axis]);
//						result_code+=buff;
//					}
//
//					{
//						int g_move_mode=-1;
//						if(pipe[i].mask==PrimitiveMask::CIRCLE)
//						{
//							g_move_mode=(pipe[i].normal*TVec<double,3>(1,1,1).GetNormalized()>0)?2:3;
//						}else
//						{
//							g_move_mode=(pipe[i].rapid||force_rapid_change)?0:1;
//						}
//						sprintf(buff,/*"N%i"*/"G0%i "/*,frame_number+=10*/,g_move_mode);
//						result_code+=buff;
//					}
//
//					//TODO ВАЖНООООО!!!! в настройках ини файла неправильно трактуются X_id=0
//					//Y_id=1
//					//Z_id=2
//					//A_id=4
//					//C_id=3
//					// почему-то перепутываются оси при измененном порядке
//
//					for(int c=0;c<5;c++)
//					{
//						if(need_print[c])
//						{
//							double new_coord_val=
//								gcode_axis_prop[c].rad_to_deg
//								?RadToDeg(new_coord[c])
//								:new_coord[c];
//
//							if(gcode_axis_prop[c].is_increment)
//							{
//								double inc_val=new_coord_val-curr_coord_machine[c];
//								sprintf(buff,gcode_axis_prop[c].format.c_str(),inc_val);//TODO из-за округления будет накапливаться ошибка DONE
//								inc_val=floor(inc_val/gcode_axis_prop[c].repeat_tol+0.5)*gcode_axis_prop[c].repeat_tol;
//
//								curr_coord_machine[c]+=inc_val;
//							}
//							else
//								sprintf(buff,gcode_axis_prop[c].format.c_str(),new_coord_val);
//							result_code+=buff;
//							curr_coord[c]=new_coord[c];
//						}
//					}
//					if(pipe[i].mask==PrimitiveMask::CIRCLE)
//					{
//						TVec<double,3> center=pipe[i].center;
//						TVec<double,3> machine_center=ToMachineToolKinematics(center,pipe[i].A,pipe[i].C);
//						TVec<double,3> machine_last_pos=ToMachineToolKinematics(pipe[i].pos,pipe[i].A,pipe[i].C);
//
//						int rot_axis;
//						bool positive;
//						IsOrthogonalVector(pipe[i].normal,rot_axis,positive);
//
//						//печатаем координаты центра кроме координаты оси вращения
//						for(int a=0;a<3;a++)
//						{
//							if(circle_interpolation_center_absol)
//							{
//								char* r[]={
//									"I=AC(%f)","J=AC(%f)","K=AC(%f)"
//								};
//								if(rot_axis!=a)
//								{
//									sprintf(buff,r[a],machine_center[a]);
//									result_code+=buff;
//								}
//							}else
//							{
//								char* r[]={
//									"I%f","J%f","K%f"
//								};
//								if(rot_axis!=a)
//								{
//									sprintf(buff,r[a],machine_center[a]-machine_last_pos[a]);
//									result_code+=buff;
//								}
//							}
//						}
//
//						if(pipe[i].spiral_times>0)
//						{
//							sprintf(buff," TURN=%i ",pipe[i].spiral_times);
//							result_code+=buff;
//						}
//					}
//				
//					if(1/*coord_repeat_tol*/<abs(pipe[i].contour_correct_feed-curr_feed)||!remove_F_repeat)
//					{
//						if(!pipe[i].rapid)
//						{
//							sprintf(buff,"F%i",pipe[i].rapid?(int)rapid_feed:(int)pipe[i].contour_correct_feed);
//							result_code+=buff;
//							curr_feed=pipe[i].feed;
//						}
//					}
//
//					result_code+="\n";
//
//					if(is_oriented_from_goto_engage_end)
//					{
//						sprintf(buff,"\nM3S%i;engage end\n",int(pipe[i].spndl_rpm));
//						result_code+=buff;
//						oriented_from_goto_engage_done=true;
//					}
//
//					if(is_oriented_from_goto_retract_end)
//					{
//						sprintf(buff,"\nM3S%i;retract end\n",int(pipe[i].spndl_rpm));
//						result_code+=buff;
//						oriented_from_goto_engage_done=false;
//					}
//
//					//if(oriented_from_goto&&oriented_from_goto_engage_done)
//					//{
//					//	if((i==0&&pipe[i].color==186)||(pipe[i-1].color!=186&&pipe[i].color==186))//спец комманда для подхода и отхода
//					//	{
//					//		sprintf(buff,"\nM5\nSPOS=%f\n",oriented_from_goto_orientation);
//					//		result_code+=buff;
//					//		oriented_from_goto_engage_done=true;
//					//	}
//					//}
//
//				//}break;
//			//case PrimitiveMask::CIRCLE:
//				/*{
//					TVec<double,3> prev_pos=pipe[i-1].pos,
//						next_pos=pipe[i].pos,
//						center;
//					if(circle_interpolation_center_absol)
//						center=prev_pos+pipe[i].center-pipe[i-1].tool_orient.pos;
//					else
//						center=pipe[i].center-pipe[i-1].tool_orient.pos;
//					sprintf(buff,"%sN%i G0%i X%.3f Y%.3f Z%.3f I%.3f J%.3f\n",
//						pipe[i].auxfun.c_str(),
//						frame_number+=10,
//						pipe[i].normal[2]<0?3:2,
//							next_pos[0],next_pos[1],next_pos[2],
//							center[0],center[1],center[2]);
//						result_code+=buff;
//
//				}break;*/
//			//default:assert(false);
//			}
//		}
//		size_t start=0;
//		int curr_frame=0;
//		do{
//			size_t curr_pos=result_code.find('\n',start);
//			if(curr_pos==string::npos)break;
//			char buff[100];
//			sprintf(buff,"\nN%i ",curr_frame);
//			curr_frame+=10;
//			result_code.replace(curr_pos,1,buff);
//			start=curr_pos+strlen(buff);
//		}while(true);
//		delete buff;
//	}
//	
//	
//void TATPProcessor::GetCode(std::vector<TToolMovementElement> &pipe, string &result_code, const char* ext_header, const char* prog_id)//только для standalone постов, далее убрать
//{
//	//boost::format header_format(G_code_header.c_str());
//	//header_format.exceptions( boost::io::all_error_bits ^ ( boost::io::too_many_args_bit | boost::io::too_few_args_bit )  );
//
//	//boost::format footer_format(G_code_footer.c_str());
//	//footer_format.exceptions( boost::io::all_error_bits ^ ( boost::io::too_many_args_bit | boost::io::too_few_args_bit )  );
//
//	//GetGCode(pipe,result_code);
//	//result_code=
//	//	(
//	//	header_format
//	//	%local_CS_G_index
//	//	%prog_id
//	//	%head_name
//	//	%tool_name
//	//	).str()
//	//	+
//	//	result_code
//	//	+
//	//	(
//	//	footer_format
//	//	%prog_id
//	//	).str();
//	//result_code=G_code_header+result_code+G_code_footer;
//	//InsertGCodeHead(result_code,TVec<double,3>(),"",false);
//}
////void GetCode(std::vector<TToolMovementElement> &pipe,string &result_code,
////	TVec<double,3> use_offset, string use_machine_offset_string,bool use_machine_offset)//только для standalone постов, далее убрать
////{
////	GetGCode(pipe,result_code);
////	result_code=(boost::format(G_code_header.c_str())%local_CS_G_index).str()+result_code+G_code_footer;
////	//result_code=G_code_header+result_code+G_code_footer;
////	//InsertGCodeHead(result_code,use_offset,use_machine_offset_string,use_machine_offset);
////}