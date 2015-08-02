#include<iostream>
#include<cstdlib>
#include<vector>
#include<string>
#include<cmath>
#include<fstream>
#include<map>
using namespace std;

#define CycleTicks 1000
//cache参数 
#define sets 8192	//组数 
#define assoc 8		//相连度 
#define blockSize 64	//cache块大小 
#define index_bias 6
#define tag_bias index_bias+13

//group参数 
#define group_size 64
#define group_num 1024
 
//各种时间延迟 
#define rw_time 1		
#define move_time 1
#define instr_delay 6
 
map<pair<long long,char> , pair<long long, char> >nextinstr;
 
enum port_type{null_PORT,RO_PORT,WO_PORT,RW_PORT};	//端口类型 
 
struct cache_t	//一个cache块 
{
  	bool valid;
  	long long tag;
  	long long time;
  	bool available;	//是否开放 
}RTcache[group_num][group_size];	

struct Set				//一个Set 
{
  	int group_id[assoc];	//代表第assoc个块的位置 
  	int domain_id[assoc];
}set[sets];		

struct GroupPort		//记录每个条带上的端口分布
{
  	int port[64];			//初始摆放位置 
  	int shift_num;		//整体位移 
}port_link[group_num];
 
int instr_num,wd_num,rd_num,wd_hit,rd_hit;  

int last_addr=-1;	//实际的指令地址
char last_type;		//实际指令类型 
int last_shift=0,last_group=0;	//实际移动的条带号和移动的位移 

int pred_addr=-1;	//预测的指令地址 
char pred_type;		//预测的指令类型 
int last_pre_shift=0, last_pre_group=0;	//预测移动的条带号和移动位移 

int pred_num=0, pred_right_num=0;		//记录预测数和正确数 

long long tick_time=0;	//时钟tick
long long latency=0;	//总的时间延迟，单位是cycle 
long long latency_read=0, latency_write=0;

long long getIndex(long long addr)	//得到index 
{
  	long long res = addr>>index_bias;
  	res %= sets;
  	return res;
}

long long getTag(long long addr)	//得到tag 
{
  	long long res = addr>>tag_bias;
  	return res;
}
void RT_shift(int group_id, int domain_id, bool is_write,bool is_predict)
{
	int ans = 128, fine_ans = 128, new_shift, dis;	
	GroupPort &this_one = port_link[group_id];
	for(int i = 0;i < 64;i++)
	{	
		if((is_write && this_one.port[i]/2) || (!is_write && this_one.port[i]%2))//找到类型匹配的端口 
		{
			dis = domain_id - (i + this_one.shift_num);
			if(abs(dis) < ans)	//选择离目标最近的 
			{
				ans = abs(dis);
				new_shift = dis;
			} 
			else if(abs(dis) == ans && abs(this_one.shift_num+dis) < fine_ans)	//优化，尽量使条带回归原来位置 
			{
				fine_ans = abs(this_one.shift_num+dis);
				new_shift = dis;
			}
		}
	}
	if(is_predict)		//给pre_shift的限制，1）预测移动的距离要小于同时的指令的移动距离
											//2）二者不是同一个条带 
	{					//这里的last_group和last_shift恰巧就是并行执行的指令的条带和移动位移 
		if(abs(new_shift)>abs(last_shift) || group_id==last_group)
		{
			pred_addr=-1; 
			return ;
		}
		last_pre_shift = new_shift;	//记录此次位移 
		last_pre_group = group_id;	//记录此次条带号 
	}
	else
	{
		//cout<<"shift: "<<new_shift<<endl;
		last_shift = new_shift;	//记录此次位移 
		last_group = group_id;	//记录此次条带号 
	}
		
	this_one.shift_num += new_shift;		//得到新位移 
	
	if(!is_predict)
	{
		tick_time += abs(new_shift) * move_time *CycleTicks;	//加上移动时间 
		latency += abs(new_shift) * move_time;
	
		tick_time += rw_time *CycleTicks;		//加上读写时间 
	
		if(is_write)
		{
			latency_write += abs(new_shift) * move_time;
		}
		else
		{
			latency_read += abs(new_shift) * move_time;
		}
	}
	return;
}
bool visitCache(long long addr, bool is_write)
{
  	long long myIndex = getIndex(addr);
  	long long myTag = getTag(addr); 
  	//cout<<"Index & Tag: "<<myIndex<<" "<<myTag<<endl;
  	int myGroup, myDomain; 
  	for(int i=0;i<assoc;i++)
  	{
		myGroup=set[myIndex].group_id[i];
		myDomain=set[myIndex].domain_id[i];
    	if(RTcache[myGroup][myDomain].valid && RTcache[myGroup][myDomain].tag == myTag)	//hit
    	{ 
	  	//cout<<"cache hit!\n";
	  	//cout<<"Group & Domain: "<<myGroup<<" "<<myDomain<<endl;
	  		RTcache[myGroup][myDomain].time=tick_time;		//更新时间 
	  		RT_shift(myGroup,myDomain,is_write,0);
      		return true;
    	}
  	}
  //cout<<"cache miss!\n";
  	int insertIndex=-1;
  	long long minTime = tick_time+1;
  	for(int i=0;i<assoc;i++)
  	{
	 	myGroup=set[myIndex].group_id[i];
	  	myDomain=set[myIndex].domain_id[i];
      	if(minTime>RTcache[myGroup][myDomain].time)	//不检查valid位了，因为不valid的time一定是0 
      	{
			minTime = RTcache[myGroup][myDomain].time;
			insertIndex = i;							//得到要插入的地方，用的是LRU替换策略 
      	}
  	}
  	int targetGroup=set[myIndex].group_id[insertIndex];	
  	int targetDomain=set[myIndex].domain_id[insertIndex];
  
  	RT_shift(targetGroup,targetDomain,is_write,0);
  
  	RTcache[targetGroup][targetDomain].valid = true;		//设置有效位 
  	RTcache[targetGroup][targetDomain].tag = myTag;		//将miss的数据写入cache
  	RTcache[targetGroup][targetDomain].time = tick_time;	//更新时间 	
  
  	return false;
}
void pre_visitCache(long long addr, bool is_write)		//只是预测，并不真正访问cache 
{
  	long long myIndex = getIndex(addr);
  	long long myTag = getTag(addr); 
  	//cout<<"Index & Tag: "<<myIndex<<" "<<myTag<<endl;
  	int myGroup, myDomain; 
  	for(int i=0;i<assoc;i++)
  	{
		myGroup=set[myIndex].group_id[i];
		myDomain=set[myIndex].domain_id[i];
    	if(RTcache[myGroup][myDomain].valid && RTcache[myGroup][myDomain].tag == myTag)	//hit
    	{ 
	  		//cout<<"cache hit!\n";
	  		//cout<<"Group & Domain: "<<myGroup<<" "<<myDomain<<endl;
	  		RT_shift(myGroup,myDomain,is_write,1);
      		return ; 
    	}
  	}
  	//cout<<"cache miss!\n";
  	int insertIndex=-1;
  	long long minTime = tick_time+1;
  	for(int i=0;i<assoc;i++)
  	{
	  	myGroup=set[myIndex].group_id[i];
	  	myDomain=set[myIndex].domain_id[i];
      	if(minTime>RTcache[myGroup][myDomain].time)	//找可用的、访问时间距离最长的 
      	{
			minTime = RTcache[myGroup][myDomain].time;
			insertIndex = i;							//得到要插入的地方，用的是LRU替换策略 
      	}
  	}
  	int targetGroup=set[myIndex].group_id[insertIndex];	
  	int targetDomain=set[myIndex].domain_id[insertIndex];
  	//cout<<"Group & Domain: "<<targetGroup<<" "<<targetDomain<<endl;
  
  	RT_shift(targetGroup,targetDomain,is_write,1);  
  	return ;		
} 
void readFile(string fileName)	//读入trace 
{
  	ifstream fin(fileName.c_str());
  	char type;
  	long long addr, instr_time;
  	string a1, a2;
  	int extra_shift=0,extra_latency=0,temp_latency=0;
  
  	while(fin >> type >> addr >> a1 >> a2 >> instr_time)	//读入请求,好像trace里面的地址是十进制的？ 
  	{
		instr_num++; 
		
		//假设判断时发现当前时间已经超过下一条指令六个延迟周期了，可以直接执行而不等待 
		if(instr_time + instr_delay * CycleTicks > tick_time)
			tick_time = instr_time + instr_delay *CycleTicks;	
	
		if(pred_addr!=-1)		//统计预测结果 
		{
			pred_num++;
			if(pred_addr==addr && pred_type==type)
			{ 
				pred_right_num++;
			}
		} 
	
    	switch(type)
    	{
      		case 'r':
				rd_num++;
				if(visitCache(addr,0))
	  				rd_hit++;
				break;
      		case 'w':
	  		case 'u':
				wd_num++;
				if(visitCache(addr,1))
	  				wd_hit++;
      			break;
      		default:
				cout << "error"<< endl;
   		}
	
		//预测当前指令的下一条，预移动条带，强制设定此处与本条指令并行完成，否则不进行预移动
		if(last_addr!=-1)	
			nextinstr[pair<long long,char>(last_addr,last_type)]=pair<long long,char>(addr,type);	//记录相邻指令对 
		last_addr=addr;
		last_type=type;
	
		if(nextinstr.count(pair<long long,char>(addr,type))!=0)	//有相应预测 
		{
			pred_addr=nextinstr[pair<long long,char>(addr,type)].first;
			pred_type=nextinstr[pair<long long,char>(addr,type)].second;
			//cout<<addr<<" "<<type<<" "<<pred_addr<<" "<<pred_type<<endl;
			if(pred_type=='r')
				pre_visitCache(pred_addr,0);
			else
				pre_visitCache(pred_addr,1);
		}	 
		else
			pred_addr=-1;
	}
}
void init_RTcache()		//初始化cache 
{
	for(int i=0;i<group_num;i++)
		for(int j=0;j<group_size;j++)
		{
			RTcache[i][j].tag=0;
			RTcache[i][j].valid=false;
			RTcache[i][j].time=0;
		}
}
void init_portmap()		//定义每个group端口个数及摆放情况
{
	/*优化1：让端口将64个domain相对平均分割开*/ 
	
	for(int i=0;i<group_num;i++)
	{
		port_link[i].shift_num = 0;
		memset(port_link[i].port,0,sizeof(port_link[i].port));
		port_link[i].port[7] = RW_PORT;
		port_link[i].port[23] = RW_PORT;
		port_link[i].port[40] = RW_PORT;
		port_link[i].port[56] = RW_PORT;
	}
} 
void init_setmap()		//定义set的划分和编址 
{
  	/*优化：set横向排列，每个set横跨1个group*/
  
  	int span=1;
  	int set_start, domain_start;
  	for(int i=0;i<sets;i++)
  	{
		set_start=(i*span)%group_num;
		domain_start=(i/(group_num/span))*(assoc/span);
		for(int j=0;j<assoc;j++)
		{
			set[i].group_id[j]=set_start+j%span;
			set[i].domain_id[j]=domain_start+j/span;
		}
  	}
}
void init_param()
{
	instr_num=wd_num=rd_num=wd_hit=rd_hit=0;  
	tick_time=0;	
	latency=0;
	latency_read=0;
	latency_write=0;	
	
	nextinstr.clear();
	last_addr=-1;	
	last_shift=0;
	last_group=0;	 

	pred_addr=-1;	
	last_pre_shift=0;
	last_pre_group=0;
	
	pred_num=0;
	pred_right_num=0;	
}
int main()
{
  
  	string trace[]={"401.bzip2","403.gcc","410.bwaves","435.gromacs","450.soplex",
  	"453.povray","456.hmmer","471.omnetpp"};
  	char output[100];
  	memset(output,'\0',sizeof(output));
  	for(int i=0;i<8;i++)
  	{
		cout<<i<<endl;
		init_param();
  		init_RTcache();
  		init_portmap();
  		init_setmap();
  		readFile("D:\\大三上\\计算机组织与体系结构\\大作业\\trace\\Result\\"+trace[i]+".trace");	
  		strcpy(output,"D:\\大三上\\计算机组织与体系结构\\大作业\\Red & Yuan\\Result4.0\\");
		strcat(output,trace[i].c_str());
  		strcat(output,".result.txt");
		ofstream fout(output);
  		fout<<"miss rate:"<<1-(double)(rd_hit+wd_hit)/(rd_num+wd_num)<<endl;
  		fout<<"on read: "<<latency_read<<" ";
  		fout<<"on write: "<<latency_write<<" ";
  		fout<<"total latency: "<<latency<<endl;  	
  	}
  	//system("pause");
  	return 0;
}
