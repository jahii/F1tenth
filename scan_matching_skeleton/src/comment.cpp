// ROS_INFO("Naive time: %d",middle_time-before_naive_time);
        // ROS_INFO("Smart time: %d",after_smart_time-middle_time);
  
        // for(int a = 0; a<1080; a++){
        //   // if(!((corresponds_smart[a].p1x==corresponds_naive[a].p1x)&&(corresponds_smart[a].p1y==corresponds_naive[a].p1y))){
        //   if(best_index_smart[a] != best_index_naive[a]){
        //     cout << a <<"_Smart index : " << best_index_smart[a] << " values : "<<corresponds_smart[a].p1x<<" "<<corresponds_smart[a].p1y<<endl;
        //     cout << "last_best : " << index_table_smart[a][0] << " low_index : "<<index_table_smart[a][1] <<" high_index : "<<index_table_smart[a][2] <<endl; 
        //     cout << a <<"_Naive index : " << best_index_naive[a] << " values : "<<corresponds_naive[a].p1x<<" "<<corresponds_naive[a].p1y<<endl;
        //     cout << " "<< endl;         
        //   }
        // }
        
        // cout << "10_N"<<corresponds_smart[100].pix << " "<< corresponds_smart[100].piy <<endl;
        // cout << "10_Naive"<<corresponds_naive[100].pix << " "<< corresponds_naive[100].piy <<endl;
        // cout << "20_N"<<corresponds_smart[200].pix << " "<< corresponds_smart[200].piy <<endl;
        // cout << "20_Naive"<<corresponds_naive[200].pix << " "<< corresponds_naive[200].piy <<endl;
        // cout << "30_N"<<corresponds_smart[300].pix << " "<< corresponds_smart[300].piy <<endl;
        // cout << "30_Naive"<<corresponds_naive[300].pix << " "<< corresponds_naive[300].piy <<endl;
        // cout << "40_N"<<corresponds_smart[400].pix << " "<< corresponds_smart[400].piy <<endl;
        // cout << "40_Naive"<<corresponds_naive[400].pix << " "<< corresponds_naive[400].piy <<endl;



// Correspond

 // if(i==30||i==300||i==500)ROS_INFO("%dth last best: %d last_low_idx: %d, last_high_idx: %d",i,last_best,last_low_idx,last_high_idx);
      
      // if((last_best < last_low_idx)&&(last_best>last_high_idx)){
      //   ROS_INFO("%dth arctan(%f/sqrt(%f^2-%f^2))=%f, point_angle = %f",i, last_best_dis,point_dis,last_best_dis,last_angle, point_ang);
      //   ROS_INFO("last_best = %d, last_low_index=%d(point_angle_index)-%d(radius index)=%d, last_high_index=%d",last_best,int(point_ang/incre),int(last_angle/incre),last_low_idx,last_high_idx);
      //   ROS_INFO("(last_low_distance,last_best_distance) = (%f, %f)",last_low_dis,last_best_dis);
      //   // ROS_INFO("size of m : %d, size of n: %d",m,n);
      //   // ROS_INFO("%dth is PROBLEM, last best: %d last_low_idx: %d, last_high_idx: %d",i,last_best,last_low_idx,last_high_idx);
      //   // ROS_INFO("point_angle_index : %d, radius_index : %d point_angle: %f,last_angle: %f",int(point_ang/incre),int(last_angle/incre),point_ang,last_angle);
      // }