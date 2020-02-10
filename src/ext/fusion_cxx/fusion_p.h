#ifndef __FUSION_P_H__
#define __FUSION_P_H__
#include "monty.h"
#include "mosektask_p.h"
#include "list"
#include "vector"
#include "unordered_map"
#include "fusion.h"
namespace mosek
{
namespace fusion
{
// mosek.fusion.BaseModel from file 'src\fusion\cxx\BaseModel_p.h'
// namespace mosek::fusion
struct p_BaseModel
{
  p_BaseModel(BaseModel * _pubthis);

  void _initialize( monty::rc_ptr<BaseModel> m);
  void _initialize( const std::string & name,
                    const std::string & licpath);

  virtual ~p_BaseModel() { /* std::cout << "~p_BaseModel()" << std::endl;*/  }

  static p_BaseModel * _get_impl(Model * _inst) { return _inst->_impl; }

  //----------------------

  bool synched;
  std::string taskname;

  monty::rc_ptr<SolutionStruct> sol_itr;
  monty::rc_ptr<SolutionStruct> sol_itg;
  monty::rc_ptr<SolutionStruct> sol_bas;

  //---------------------

  std::unique_ptr<Task> task;

  //---------------------
  void task_setLogHandler (const msghandler_t & handler);
  void task_setDataCallbackHandler (const datacbhandler_t & handler);
  void task_setCallbackHandler (const cbhandler_t & handler);

  int task_append_barvar(int size, int num);

  void task_var_name   (int index, const std::string & name);
  void task_con_name   (int index, const std::string & name);
  void task_cone_name  (int index, const std::string & name);
  void task_barvar_name(int index, const std::string & name);
  void task_objectivename(         const std::string & name);

  void task_format_var_names(const std::shared_ptr<monty::ndarray<int,1>> subj, const std::string & format,const std::shared_ptr<monty::ndarray<int,1>> dims, const std::shared_ptr<monty::ndarray<long long,1>> sp);
  void task_format_con_names(const std::shared_ptr<monty::ndarray<int,1>> subi, const std::string & format,const std::shared_ptr<monty::ndarray<int,1>> dims, const std::shared_ptr<monty::ndarray<long long,1>> sp);
  void task_format_cone_names(const std::shared_ptr<monty::ndarray<int,1>> subk, const std::string & format,const std::shared_ptr<monty::ndarray<int,1>> dims, const std::shared_ptr<monty::ndarray<long long,1>> sp);

  void task_break_solve();

  //--------------------------

  int task_numvar();
  int task_numcon();
  int task_numcone();
  int task_numbarvar();

  //--------------------------

  void task_put_param(const std::string & name, const std::string & value);
  void task_put_param(const std::string & name, int    value);
  void task_put_param(const std::string & name, double value);
  
  double    task_get_dinf (const std::string & name);
  int       task_get_iinf (const std::string & name);
  long long task_get_liinf(const std::string & name);
  
  //--------------------------

  void task_con_putboundlist_fr(const std::shared_ptr<monty::ndarray<int,1>> idxs);
  void task_con_putboundlist_lo(const std::shared_ptr<monty::ndarray<int,1>> idxs, const std::shared_ptr<monty::ndarray<double,1>> & rhs);
  void task_con_putboundlist_up(const std::shared_ptr<monty::ndarray<int,1>> idxs, const std::shared_ptr<monty::ndarray<double,1>> & rhs);
  void task_con_putboundlist_fx(const std::shared_ptr<monty::ndarray<int,1>> idxs, const std::shared_ptr<monty::ndarray<double,1>> & rhs);
  void task_con_putboundlist_ra(const std::shared_ptr<monty::ndarray<int,1>> idxs, const std::shared_ptr<monty::ndarray<double,1>> & lb , 
                                const std::shared_ptr<monty::ndarray<double,1>> & ub );

  void task_var_putboundlist_fr(const std::shared_ptr<monty::ndarray<int,1>> idxs);
  void task_var_putboundlist_lo(const std::shared_ptr<monty::ndarray<int,1>> idxs, const std::shared_ptr<monty::ndarray<double,1>> & rhs);
  void task_var_putboundlist_up(const std::shared_ptr<monty::ndarray<int,1>> idxs, const std::shared_ptr<monty::ndarray<double,1>> & rhs);
  void task_var_putboundlist_fx(const std::shared_ptr<monty::ndarray<int,1>> idxs, const std::shared_ptr<monty::ndarray<double,1>> & rhs);
  void task_var_putboundlist_ra(const std::shared_ptr<monty::ndarray<int,1>> idxs, const std::shared_ptr<monty::ndarray<double,1>> & lb , 
                                const std::shared_ptr<monty::ndarray<double,1>> & ub );
  
  void task_var_putintlist(const std::shared_ptr<monty::ndarray<int,1>> & idxs);
  void task_var_putcontlist(const std::shared_ptr<monty::ndarray<int,1>> & idxs); 
 
  //--------------------------

  void task_putbararowlist(const std::shared_ptr<monty::ndarray<int,1>>       subi,
                           const std::shared_ptr<monty::ndarray<long long,1>> ptr,
                           const std::shared_ptr<monty::ndarray<int,1>>       subj,
                           const std::shared_ptr<monty::ndarray<long long,1>> matidx);

  void task_putbaraijlist(const std::shared_ptr<monty::ndarray<int,1>> subi,
                          const std::shared_ptr<monty::ndarray<int,1>> subj,
                          std::shared_ptr<monty::ndarray<long long,1>> matidx);
  
  void task_putbarc(const std::shared_ptr<monty::ndarray<int,1>> subj,
                    const std::shared_ptr<monty::ndarray<int,1>> subl,
                    const std::shared_ptr<monty::ndarray<int,1>> subk,
                    const std::shared_ptr<monty::ndarray<double,1>> val);
  
  std::shared_ptr<monty::ndarray<long long,1>> task_appendsymmatlist (const std::shared_ptr<monty::ndarray<int,1>>       & dim, 
                                                                      const std::shared_ptr<monty::ndarray<long long,1>> & nz, 
                                                                      const std::shared_ptr<monty::ndarray<int,1>>       & subk, 
                                                                      const std::shared_ptr<monty::ndarray<int,1>>       & subl, 
                                                                      const std::shared_ptr<monty::ndarray<double,1>>    & val);
  int  task_barvar_dim(int j);
  void task_putbaraij (int i, int j, int k);
  void task_putbaraij (int i, int j, const std::shared_ptr<monty::ndarray<int,1>> & k);
  void task_putbarcj  (int j, int k);
  void task_putbarcj  (int j,        const std::shared_ptr<monty::ndarray<int,1>> & k);
  int  task_barvardim (int index);

  int task_append_var(int num);
  int task_append_con(int num);

  void task_append_zerocones (int numcone);
  void task_clear_cones   ( const std::shared_ptr<monty::ndarray<int,1>> & idxs );
  void task_put_zerocones ( const std::shared_ptr<monty::ndarray<int,1>> & idxs, int conesize, int numcone,  const std::shared_ptr<monty::ndarray<int,1>> & membs );
  void task_put_quadcones ( const std::shared_ptr<monty::ndarray<int,1>> & idxs, int conesize, int numcone,  const std::shared_ptr<monty::ndarray<int,1>> & membs );
  void task_put_rquadcones( const std::shared_ptr<monty::ndarray<int,1>> & idxs, int conesize, int numcone,  const std::shared_ptr<monty::ndarray<int,1>> & membs );
  void task_put_pexpcones ( const std::shared_ptr<monty::ndarray<int,1>> & idxs, int conesize, int numcone,  const std::shared_ptr<monty::ndarray<int,1>> & membs );
  void task_put_ppowcones ( const std::shared_ptr<monty::ndarray<int,1>> & idxs, int conesize, int numcone,  const std::shared_ptr<monty::ndarray<int,1>> & membs, const std::shared_ptr<monty::ndarray<double,1>> & alpha );
  void task_put_dexpcones ( const std::shared_ptr<monty::ndarray<int,1>> & idxs, int conesize, int numcone,  const std::shared_ptr<monty::ndarray<int,1>> & membs );
  void task_put_dpowcones ( const std::shared_ptr<monty::ndarray<int,1>> & idxs, int conesize, int numcone,  const std::shared_ptr<monty::ndarray<int,1>> & membs, const std::shared_ptr<monty::ndarray<double,1>> & alpha );

  void task_putarowlist(
    const std::shared_ptr<monty::ndarray<int,1>>       & idxs, 
    const std::shared_ptr<monty::ndarray<long long,1>> & ptrb, 
    const std::shared_ptr<monty::ndarray<int,1>>       & subj, 
    const std::shared_ptr<monty::ndarray<double,1>>    & cof);
  void task_putaijlist(
    const std::shared_ptr<monty::ndarray<int,1>>       & subi, 
    const std::shared_ptr<monty::ndarray<int,1>>       & subj, 
    const std::shared_ptr<monty::ndarray<double,1>>    & cof, 
    long long                           num);

  void task_setnumvar(int num);
  void task_cleanup(int oldnum, int oldnumcon, int oldnumcone, int oldnumbarvar);
  void task_solve(bool remote, const std::string & server, const std::string & port);

  void task_putobjective( 
    bool                             maximize,
    const std::shared_ptr<monty::ndarray<int,1>>    & subj    ,
    const std::shared_ptr<monty::ndarray<double,1>> & cof     ,
    double                           cfix    );

  void task_putclist(   
    const std::shared_ptr<monty::ndarray<int,1>>    & subj,
    const std::shared_ptr<monty::ndarray<double,1>> & cof);


  void task_putobjectivename(const std::string & name);

  void task_write(const std::string & filename);
  void task_dump (const std::string & filename);

  MSKtask_t task_get();
  MSKtask_t __mosek_2fusion_2BaseModel__task_get();
  
  void dispose();

  void task_putxx_slice(SolutionType which, int first, int last, std::shared_ptr<monty::ndarray<double,1>> & xx);

  static void env_syeig (int n, std::shared_ptr<monty::ndarray<double,1>> & a, std::shared_ptr<monty::ndarray<double,1>> & w);
  static void env_potrf (int n, std::shared_ptr<monty::ndarray<double,1>> & a);                        
  static void env_syevd (int n, std::shared_ptr<monty::ndarray<double,1>> & a, std::shared_ptr<monty::ndarray<double,1>> & w);

  static void env_putlicensecode(std::shared_ptr<monty::ndarray<int,1>> code);
  static void env_putlicensepath(const std::string &licfile);
  static void env_putlicensewait(int wait);

  static std::string env_getversion();

  void convertSolutionStatus(MSKsoltypee soltype, p_SolutionStruct * sol, MSKsolstae status, MSKprostae prosta);


};

// End of file 'src\fusion\cxx\BaseModel_p.h'
struct p_Model : public ::mosek::fusion::p_BaseModel
{
Model * _pubthis;
static mosek::fusion::p_Model* _get_impl(mosek::fusion::Model * _inst){ return static_cast< mosek::fusion::p_Model* >(mosek::fusion::p_BaseModel::_get_impl(_inst)); }
static mosek::fusion::p_Model * _get_impl(mosek::fusion::Model::t _inst) { return _get_impl(_inst.get()); }
p_Model(Model * _pubthis);
virtual ~p_Model() { /* std::cout << "~p_Model" << std::endl;*/ };
monty::rc_ptr< ::mosek::fusion::WorkStack > xs{};monty::rc_ptr< ::mosek::fusion::WorkStack > ws{};monty::rc_ptr< ::mosek::fusion::WorkStack > rs{};monty::rc_ptr< ::mosek::fusion::Utils::StringIntMap > con_map{};std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::ModelConstraint >,1 > > cons{};std::shared_ptr< monty::ndarray< int,1 > > natconmap_type{};std::shared_ptr< monty::ndarray< double,1 > > natconmap_ub{};std::shared_ptr< monty::ndarray< double,1 > > natconmap_lb{};std::shared_ptr< monty::ndarray< double,1 > > natconmap_efix{};std::shared_ptr< monty::ndarray< int,1 > > natconmap_idx{};std::shared_ptr< monty::ndarray< long long,1 > > natconmap_slackidx{};std::shared_ptr< monty::ndarray< int,1 > > natconmap_blockid{};monty::rc_ptr< ::mosek::fusion::LinkedBlocks > natconmap{};std::shared_ptr< monty::ndarray< bool,1 > > initsol_xx_flag{};std::shared_ptr< monty::ndarray< double,1 > > initsol_xx{};monty::rc_ptr< ::mosek::fusion::Utils::StringIntMap > var_map{};std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::ModelVariable >,1 > > barvars{};std::shared_ptr< monty::ndarray< int,1 > > natbarvarmap_ptr{};std::shared_ptr< monty::ndarray< int,1 > > natbarvarmap_num{};int natbarvarmap_nblock{};std::shared_ptr< monty::ndarray< int,1 > > natbarvar_dim{};std::shared_ptr< monty::ndarray< long long,1 > > natbarvar_ptr{};int natbarvar_numbarvarelm{};std::shared_ptr< monty::ndarray< int,1 > > natbarvar_j{};std::shared_ptr< monty::ndarray< int,1 > > natbarvar_i{};std::shared_ptr< monty::ndarray< int,1 > > natbarvar_idx{};std::shared_ptr< monty::ndarray< int,1 > > natvarmap_type{};std::shared_ptr< monty::ndarray< int,1 > > natconemap_dim{};monty::rc_ptr< ::mosek::fusion::LinkedBlocks > natconemap{};std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::ModelVariable >,1 > > vars{};int bfixidx{};std::shared_ptr< monty::ndarray< int,1 > > natvarmap_idx{};std::shared_ptr< monty::ndarray< int,1 > > natvarmap_blockid{};monty::rc_ptr< ::mosek::fusion::LinkedBlocks > natvarmap{};mosek::fusion::SolutionType solutionptr{};mosek::fusion::AccSolutionStatus acceptable_sol{};std::string model_name{};virtual void destroy();
static Model::t _new_Model(monty::rc_ptr< ::mosek::fusion::Model > _467);
void _initialize(monty::rc_ptr< ::mosek::fusion::Model > _467);
static Model::t _new_Model(const std::string &  _472);
void _initialize(const std::string &  _472);
static Model::t _new_Model();
void _initialize();
virtual monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > __mosek_2fusion_2Model__formstConstr(monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > _475,std::shared_ptr< monty::ndarray< int,1 > > _476,std::shared_ptr< monty::ndarray< int,1 > > _477) ;
virtual void connames(std::shared_ptr< monty::ndarray< int,1 > > _478,const std::string &  _479,std::shared_ptr< monty::ndarray< int,1 > > _480,std::shared_ptr< monty::ndarray< long long,1 > > _481) ;
virtual void varnames(std::shared_ptr< monty::ndarray< int,1 > > _482,const std::string &  _483,std::shared_ptr< monty::ndarray< int,1 > > _484,std::shared_ptr< monty::ndarray< long long,1 > > _485) ;
virtual void varname(int _486,const std::string &  _487) ;
virtual void natbarvarmap_get(int _488,std::shared_ptr< monty::ndarray< int,1 > > _489) ;
virtual void natbarvar_get(int _493,std::shared_ptr< monty::ndarray< long long,1 > > _494) ;
virtual int natbarvarmap_alloc(int _501,int _502) ;
virtual int natvarmap_alloc(int _520) ;
virtual int natconmap_alloc(int _530) ;
virtual int natconemap_alloc(int _540) ;
virtual void make_continuous(std::shared_ptr< monty::ndarray< long long,1 > > _543) ;
virtual void make_integer(std::shared_ptr< monty::ndarray< long long,1 > > _549) ;
static  void putlicensewait(bool _555);
static  void putlicensepath(const std::string &  _556);
static  void putlicensecode(std::shared_ptr< monty::ndarray< int,1 > > _557);
virtual /* override */ void dispose() ;
virtual void nativeVarToStr(int _560,monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > _561) ;
virtual MSKtask_t __mosek_2fusion_2Model__getTask() ;
virtual void getConstraintValues(bool _562,std::shared_ptr< monty::ndarray< int,1 > > _563,std::shared_ptr< monty::ndarray< double,1 > > _564,int _565) ;
virtual void getVariableDuals(bool _573,std::shared_ptr< monty::ndarray< long long,1 > > _574,std::shared_ptr< monty::ndarray< double,1 > > _575,int _576) ;
virtual void getVariableValues(bool _582,std::shared_ptr< monty::ndarray< long long,1 > > _583,std::shared_ptr< monty::ndarray< double,1 > > _584,int _585) ;
virtual void setVariableValues(bool _591,std::shared_ptr< monty::ndarray< long long,1 > > _592,std::shared_ptr< monty::ndarray< double,1 > > _593) ;
virtual void flushNames() ;
virtual void writeTask(const std::string &  _603) ;
virtual long long getSolverLIntInfo(const std::string &  _604) ;
virtual int getSolverIntInfo(const std::string &  _605) ;
virtual double getSolverDoubleInfo(const std::string &  _606) ;
virtual void setCallbackHandler(mosek::cbhandler_t  _607) ;
virtual void setDataCallbackHandler(mosek::datacbhandler_t  _608) ;
virtual void setLogHandler(mosek::msghandler_t  _609) ;
virtual void setSolverParam(const std::string &  _610,double _611) ;
virtual void setSolverParam(const std::string &  _612,int _613) ;
virtual void setSolverParam(const std::string &  _614,const std::string &  _615) ;
virtual void breakSolver() ;
virtual void solve(const std::string &  _616,const std::string &  _617) ;
virtual void solve() ;
virtual void flushSolutions() ;
virtual void flush_initsol(mosek::fusion::SolutionType _618) ;
virtual mosek::fusion::SolutionStatus getDualSolutionStatus() ;
virtual mosek::fusion::ProblemStatus getProblemStatus() ;
virtual mosek::fusion::SolutionStatus getPrimalSolutionStatus() ;
virtual double dualObjValue() ;
virtual double primalObjValue() ;
virtual monty::rc_ptr< ::mosek::fusion::SolutionStruct > __mosek_2fusion_2Model__get_sol_cache(mosek::fusion::SolutionType _625,bool _626,bool _627) ;
virtual monty::rc_ptr< ::mosek::fusion::SolutionStruct > __mosek_2fusion_2Model__get_sol_cache(mosek::fusion::SolutionType _633,bool _634) ;
virtual void setSolution_xx(std::shared_ptr< monty::ndarray< int,1 > > _635,std::shared_ptr< monty::ndarray< double,1 > > _636) ;
virtual void ensure_initsol_xx() ;
virtual std::shared_ptr< monty::ndarray< double,1 > > getSolution_bars(mosek::fusion::SolutionType _643) ;
virtual std::shared_ptr< monty::ndarray< double,1 > > getSolution_barx(mosek::fusion::SolutionType _644) ;
virtual std::shared_ptr< monty::ndarray< double,1 > > getSolution_y(mosek::fusion::SolutionType _645) ;
virtual std::shared_ptr< monty::ndarray< double,1 > > getSolution_xc(mosek::fusion::SolutionType _646) ;
virtual std::shared_ptr< monty::ndarray< double,1 > > getSolution_snx(mosek::fusion::SolutionType _647) ;
virtual std::shared_ptr< monty::ndarray< double,1 > > getSolution_suc(mosek::fusion::SolutionType _648) ;
virtual std::shared_ptr< monty::ndarray< double,1 > > getSolution_slc(mosek::fusion::SolutionType _649) ;
virtual std::shared_ptr< monty::ndarray< double,1 > > getSolution_sux(mosek::fusion::SolutionType _650) ;
virtual std::shared_ptr< monty::ndarray< double,1 > > getSolution_slx(mosek::fusion::SolutionType _651) ;
virtual std::shared_ptr< monty::ndarray< double,1 > > getSolution_yx(mosek::fusion::SolutionType _652) ;
virtual std::shared_ptr< monty::ndarray< double,1 > > getSolution_xx(mosek::fusion::SolutionType _653) ;
virtual void selectedSolution(mosek::fusion::SolutionType _654) ;
virtual mosek::fusion::AccSolutionStatus getAcceptedSolutionStatus() ;
virtual void acceptedSolutionStatus(mosek::fusion::AccSolutionStatus _655) ;
virtual mosek::fusion::ProblemStatus getProblemStatus(mosek::fusion::SolutionType _656) ;
virtual mosek::fusion::SolutionStatus getDualSolutionStatus(mosek::fusion::SolutionType _658) ;
virtual mosek::fusion::SolutionStatus getPrimalSolutionStatus(mosek::fusion::SolutionType _659) ;
virtual mosek::fusion::SolutionStatus getSolutionStatus(mosek::fusion::SolutionType _660,bool _661) ;
virtual void update(std::shared_ptr< monty::ndarray< int,1 > > _664,monty::rc_ptr< ::mosek::fusion::Expression > _665) ;
virtual void update(std::shared_ptr< monty::ndarray< int,1 > > _698,monty::rc_ptr< ::mosek::fusion::Expression > _699,std::shared_ptr< monty::ndarray< int,1 > > _700,bool _701) ;
virtual void updateObjective(monty::rc_ptr< ::mosek::fusion::Expression > _731,monty::rc_ptr< ::mosek::fusion::Variable > _732) ;
virtual void objective_(const std::string &  _766,mosek::fusion::ObjectiveSense _767,monty::rc_ptr< ::mosek::fusion::Expression > _768) ;
virtual void objective(double _797) ;
virtual void objective(mosek::fusion::ObjectiveSense _798,double _799) ;
virtual void objective(mosek::fusion::ObjectiveSense _800,monty::rc_ptr< ::mosek::fusion::Expression > _801) ;
virtual void objective(const std::string &  _802,double _803) ;
virtual void objective(const std::string &  _804,mosek::fusion::ObjectiveSense _805,double _806) ;
virtual void objective(const std::string &  _807,mosek::fusion::ObjectiveSense _808,monty::rc_ptr< ::mosek::fusion::Expression > _809) ;
virtual monty::rc_ptr< ::mosek::fusion::Constraint > __mosek_2fusion_2Model__constraint(monty::rc_ptr< ::mosek::fusion::Expression > _810,monty::rc_ptr< ::mosek::fusion::ConeDomain > _811) ;
virtual monty::rc_ptr< ::mosek::fusion::Constraint > __mosek_2fusion_2Model__constraint(const std::string &  _812,monty::rc_ptr< ::mosek::fusion::Expression > _813,monty::rc_ptr< ::mosek::fusion::ConeDomain > _814) ;
virtual monty::rc_ptr< ::mosek::fusion::Constraint > __mosek_2fusion_2Model__constraint(monty::rc_ptr< ::mosek::fusion::Expression > _815,monty::rc_ptr< ::mosek::fusion::RangeDomain > _816) ;
virtual monty::rc_ptr< ::mosek::fusion::Constraint > __mosek_2fusion_2Model__constraint(const std::string &  _817,monty::rc_ptr< ::mosek::fusion::Expression > _818,monty::rc_ptr< ::mosek::fusion::RangeDomain > _819) ;
virtual monty::rc_ptr< ::mosek::fusion::Constraint > __mosek_2fusion_2Model__constraint(monty::rc_ptr< ::mosek::fusion::Expression > _820,monty::rc_ptr< ::mosek::fusion::LinearDomain > _821) ;
virtual monty::rc_ptr< ::mosek::fusion::Constraint > __mosek_2fusion_2Model__constraint(const std::string &  _822,monty::rc_ptr< ::mosek::fusion::Expression > _823,monty::rc_ptr< ::mosek::fusion::LinearDomain > _824) ;
virtual monty::rc_ptr< ::mosek::fusion::Constraint > __mosek_2fusion_2Model__constraint(monty::rc_ptr< ::mosek::fusion::Expression > _825,monty::rc_ptr< ::mosek::fusion::LinPSDDomain > _826) ;
virtual monty::rc_ptr< ::mosek::fusion::Constraint > __mosek_2fusion_2Model__constraint(const std::string &  _827,monty::rc_ptr< ::mosek::fusion::Expression > _828,monty::rc_ptr< ::mosek::fusion::LinPSDDomain > _829) ;
virtual monty::rc_ptr< ::mosek::fusion::Constraint > __mosek_2fusion_2Model__constraint(monty::rc_ptr< ::mosek::fusion::Expression > _830,monty::rc_ptr< ::mosek::fusion::PSDDomain > _831) ;
virtual monty::rc_ptr< ::mosek::fusion::Constraint > __mosek_2fusion_2Model__constraint(const std::string &  _832,monty::rc_ptr< ::mosek::fusion::Expression > _833,monty::rc_ptr< ::mosek::fusion::PSDDomain > _834) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(monty::rc_ptr< ::mosek::fusion::LinPSDDomain > _835) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(int _836,int _837,monty::rc_ptr< ::mosek::fusion::LinPSDDomain > _838) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(int _839,monty::rc_ptr< ::mosek::fusion::LinPSDDomain > _840) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(const std::string &  _841,monty::rc_ptr< ::mosek::fusion::LinPSDDomain > _842) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(const std::string &  _843,int _844,int _845,monty::rc_ptr< ::mosek::fusion::LinPSDDomain > _846) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(const std::string &  _847,int _848,monty::rc_ptr< ::mosek::fusion::LinPSDDomain > _849) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(const std::string &  _850,std::shared_ptr< monty::ndarray< int,1 > > _851,monty::rc_ptr< ::mosek::fusion::LinPSDDomain > _852) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(monty::rc_ptr< ::mosek::fusion::PSDDomain > _853) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(int _854,int _855,monty::rc_ptr< ::mosek::fusion::PSDDomain > _856) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(int _857,monty::rc_ptr< ::mosek::fusion::PSDDomain > _858) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(const std::string &  _859,monty::rc_ptr< ::mosek::fusion::PSDDomain > _860) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(const std::string &  _861,int _862,int _863,monty::rc_ptr< ::mosek::fusion::PSDDomain > _864) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(const std::string &  _865,int _866,monty::rc_ptr< ::mosek::fusion::PSDDomain > _867) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(const std::string &  _868,std::shared_ptr< monty::ndarray< int,1 > > _869,monty::rc_ptr< ::mosek::fusion::PSDDomain > _870) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(monty::rc_ptr< ::mosek::fusion::ConeDomain > _871) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(monty::rc_ptr< ::mosek::fusion::RangeDomain > _872) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(monty::rc_ptr< ::mosek::fusion::LinearDomain > _873) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(std::shared_ptr< monty::ndarray< int,1 > > _874,monty::rc_ptr< ::mosek::fusion::ConeDomain > _875) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(std::shared_ptr< monty::ndarray< int,1 > > _876,monty::rc_ptr< ::mosek::fusion::RangeDomain > _877) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(std::shared_ptr< monty::ndarray< int,1 > > _878,monty::rc_ptr< ::mosek::fusion::LinearDomain > _879) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(std::shared_ptr< monty::ndarray< int,1 > > _880) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(int _881,monty::rc_ptr< ::mosek::fusion::ConeDomain > _882) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(int _883,monty::rc_ptr< ::mosek::fusion::RangeDomain > _884) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(int _885,monty::rc_ptr< ::mosek::fusion::LinearDomain > _886) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(int _887) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable() ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(const std::string &  _888,monty::rc_ptr< ::mosek::fusion::ConeDomain > _889) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(const std::string &  _890,monty::rc_ptr< ::mosek::fusion::RangeDomain > _891) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(const std::string &  _892,monty::rc_ptr< ::mosek::fusion::LinearDomain > _893) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(const std::string &  _894,std::shared_ptr< monty::ndarray< int,1 > > _895,monty::rc_ptr< ::mosek::fusion::ConeDomain > _896) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(const std::string &  _897,std::shared_ptr< monty::ndarray< int,1 > > _898,monty::rc_ptr< ::mosek::fusion::RangeDomain > _899) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(const std::string &  _900,std::shared_ptr< monty::ndarray< int,1 > > _901,monty::rc_ptr< ::mosek::fusion::LinearDomain > _902) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(const std::string &  _903,std::shared_ptr< monty::ndarray< int,1 > > _904) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(const std::string &  _905,int _906,monty::rc_ptr< ::mosek::fusion::ConeDomain > _907) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(const std::string &  _908,int _909,monty::rc_ptr< ::mosek::fusion::RangeDomain > _910) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(const std::string &  _911,int _912,monty::rc_ptr< ::mosek::fusion::LinearDomain > _913) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(const std::string &  _914,int _915) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(const std::string &  _916) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__ranged_variable(const std::string &  _917,std::shared_ptr< monty::ndarray< int,1 > > _918,monty::rc_ptr< ::mosek::fusion::RangeDomain > _919) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable_(const std::string &  _940,std::shared_ptr< monty::ndarray< int,1 > > _941,monty::rc_ptr< ::mosek::fusion::ConeDomain > _942) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable_(const std::string &  _970,std::shared_ptr< monty::ndarray< int,1 > > _971,monty::rc_ptr< ::mosek::fusion::LinearDomain > _972) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable_(const std::string &  _1014,std::shared_ptr< monty::ndarray< int,1 > > _1015,monty::rc_ptr< ::mosek::fusion::LinPSDDomain > _1016) ;
virtual monty::rc_ptr< ::mosek::fusion::SymmetricVariable > __mosek_2fusion_2Model__variable_(const std::string &  _1038,std::shared_ptr< monty::ndarray< int,1 > > _1039,monty::rc_ptr< ::mosek::fusion::PSDDomain > _1040) ;
virtual monty::rc_ptr< ::mosek::fusion::Constraint > __mosek_2fusion_2Model__constraint_(const std::string &  _1055,monty::rc_ptr< ::mosek::fusion::Expression > _1056,monty::rc_ptr< ::mosek::fusion::RangeDomain > _1057) ;
virtual void update_bfix(std::shared_ptr< monty::ndarray< int,1 > > _1095,std::shared_ptr< monty::ndarray< double,1 > > _1096) ;
virtual void putarows(std::shared_ptr< monty::ndarray< int,1 > > _1098,monty::rc_ptr< ::mosek::fusion::WorkStack > _1099,int _1100,int _1101,int _1102,int _1103,int _1104,int _1105,std::shared_ptr< monty::ndarray< int,1 > > _1106) ;
virtual monty::rc_ptr< ::mosek::fusion::Constraint > __mosek_2fusion_2Model__constraint_(const std::string &  _1147,monty::rc_ptr< ::mosek::fusion::Expression > _1148,monty::rc_ptr< ::mosek::fusion::PSDDomain > _1149) ;
virtual monty::rc_ptr< ::mosek::fusion::Constraint > __mosek_2fusion_2Model__constraint_(const std::string &  _1214,monty::rc_ptr< ::mosek::fusion::Expression > _1215,monty::rc_ptr< ::mosek::fusion::LinPSDDomain > _1216) ;
virtual monty::rc_ptr< ::mosek::fusion::Constraint > __mosek_2fusion_2Model__constraint_(const std::string &  _1266,monty::rc_ptr< ::mosek::fusion::Expression > _1267,monty::rc_ptr< ::mosek::fusion::ConeDomain > _1268) ;
virtual monty::rc_ptr< ::mosek::fusion::Constraint > __mosek_2fusion_2Model__constraint_(const std::string &  _1315,monty::rc_ptr< ::mosek::fusion::Expression > _1316,monty::rc_ptr< ::mosek::fusion::LinearDomain > _1317) ;
static  std::string getVersion();
virtual bool hasConstraint(const std::string &  _1362) ;
virtual bool hasVariable(const std::string &  _1363) ;
virtual monty::rc_ptr< ::mosek::fusion::Constraint > __mosek_2fusion_2Model__getConstraint(int _1364) ;
virtual monty::rc_ptr< ::mosek::fusion::Constraint > __mosek_2fusion_2Model__getConstraint(const std::string &  _1365) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__getVariable(int _1366) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__getVariable(const std::string &  _1367) ;
virtual std::string getName() ;
virtual monty::rc_ptr< ::mosek::fusion::Model > __mosek_2fusion_2Model__clone() ;
}; // struct Model;

// mosek.fusion.Debug from file 'src\fusion\cxx\Debug_p.h'
// namespace mosek::fusion
struct p_Debug
{
  Debug * _pubthis;

  p_Debug(Debug * _pubthis) : _pubthis(_pubthis) {}

  static Debug::t o ()                 { return monty::rc_ptr<Debug>(new Debug()); }
  Debug::t p (const std::string & val) { std::cout << val; return Debug::t(_pubthis); }
  Debug::t p (      int val)           { std::cout << val; return Debug::t(_pubthis); }
  Debug::t p (long long val)           { std::cout << val; return Debug::t(_pubthis); }
  Debug::t p (   double val)           { std::cout << val; return Debug::t(_pubthis); }
  Debug::t p (     bool val)           { std::cout << val; return Debug::t(_pubthis); }
  Debug::t lf()                        { std::cout << std::endl; return Debug::t(_pubthis); }

  static p_Debug * _get_impl(Debug * _inst) { return _inst->ptr; }

  template<typename T>
  Debug::t p(const std::shared_ptr<monty::ndarray<T,1>> & val)
  {
    if (val->size() > 0)
    {
      std::cout << (*val)[0];
      for (int i = 1; i < val->size(); ++i)
        std::cout << "," << (*val)[i];
    }
    return Debug::t(_pubthis);
  }

  Debug::t __mosek_2fusion_2Debug__p (const std::string & val) { std::cout << val; return Debug::t(_pubthis); }

  template<class C>
  Debug::t __mosek_2fusion_2Debug__p (C val) { std::cout << val; return Debug::t(_pubthis); }
  Debug::t __mosek_2fusion_2Debug__lf()      { std::cout << std::endl; return Debug::t(_pubthis); }
  
};
// End of file 'src\fusion\cxx\Debug_p.h'
struct p_Sort
{
Sort * _pubthis;
static mosek::fusion::p_Sort* _get_impl(mosek::fusion::Sort * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_Sort * _get_impl(mosek::fusion::Sort::t _inst) { return _get_impl(_inst.get()); }
p_Sort(Sort * _pubthis);
virtual ~p_Sort() { /* std::cout << "~p_Sort" << std::endl;*/ };
virtual void destroy();
static  void argTransposeSort(std::shared_ptr< monty::ndarray< long long,1 > > _154,std::shared_ptr< monty::ndarray< long long,1 > > _155,int _156,int _157,int _158,std::shared_ptr< monty::ndarray< long long,1 > > _159);
static  void argsort(std::shared_ptr< monty::ndarray< long long,1 > > _167,std::shared_ptr< monty::ndarray< long long,1 > > _168);
static  void argsort(std::shared_ptr< monty::ndarray< long long,1 > > _169,std::shared_ptr< monty::ndarray< int,1 > > _170);
static  void argsort(std::shared_ptr< monty::ndarray< long long,1 > > _171,std::shared_ptr< monty::ndarray< long long,1 > > _172,std::shared_ptr< monty::ndarray< long long,1 > > _173);
static  void argsort(std::shared_ptr< monty::ndarray< long long,1 > > _174,std::shared_ptr< monty::ndarray< int,1 > > _175,std::shared_ptr< monty::ndarray< int,1 > > _176);
static  void argsort(std::shared_ptr< monty::ndarray< long long,1 > > _177,std::shared_ptr< monty::ndarray< long long,1 > > _178,long long _179,long long _180);
static  void argsort(std::shared_ptr< monty::ndarray< long long,1 > > _181,std::shared_ptr< monty::ndarray< int,1 > > _182,long long _183,long long _184);
static  void argsort(std::shared_ptr< monty::ndarray< long long,1 > > _185,std::shared_ptr< monty::ndarray< long long,1 > > _186,std::shared_ptr< monty::ndarray< long long,1 > > _187,long long _188,long long _189);
static  void argsort(std::shared_ptr< monty::ndarray< long long,1 > > _190,std::shared_ptr< monty::ndarray< int,1 > > _191,std::shared_ptr< monty::ndarray< int,1 > > _192,long long _193,long long _194);
static  void argsort(std::shared_ptr< monty::ndarray< long long,1 > > _195,std::shared_ptr< monty::ndarray< long long,1 > > _196,long long _197,long long _198,bool _199);
static  void argsort(std::shared_ptr< monty::ndarray< long long,1 > > _202,std::shared_ptr< monty::ndarray< int,1 > > _203,long long _204,long long _205,bool _206);
static  void argsort(std::shared_ptr< monty::ndarray< long long,1 > > _209,std::shared_ptr< monty::ndarray< long long,1 > > _210,std::shared_ptr< monty::ndarray< long long,1 > > _211,long long _212,long long _213,bool _214);
static  void argsort(std::shared_ptr< monty::ndarray< long long,1 > > _217,std::shared_ptr< monty::ndarray< int,1 > > _218,std::shared_ptr< monty::ndarray< int,1 > > _219,long long _220,long long _221,bool _222);
static  void argbucketsort(std::shared_ptr< monty::ndarray< long long,1 > > _225,std::shared_ptr< monty::ndarray< long long,1 > > _226,long long _227,long long _228,long long _229,long long _230);
static  void argbucketsort(std::shared_ptr< monty::ndarray< long long,1 > > _231,std::shared_ptr< monty::ndarray< int,1 > > _232,long long _233,long long _234,int _235,int _236);
static  void getminmax(std::shared_ptr< monty::ndarray< long long,1 > > _237,std::shared_ptr< monty::ndarray< long long,1 > > _238,std::shared_ptr< monty::ndarray< long long,1 > > _239,long long _240,long long _241,std::shared_ptr< monty::ndarray< long long,1 > > _242);
static  void getminmax(std::shared_ptr< monty::ndarray< long long,1 > > _245,std::shared_ptr< monty::ndarray< int,1 > > _246,std::shared_ptr< monty::ndarray< int,1 > > _247,long long _248,long long _249,std::shared_ptr< monty::ndarray< int,1 > > _250);
static  bool issorted(std::shared_ptr< monty::ndarray< long long,1 > > _253,std::shared_ptr< monty::ndarray< long long,1 > > _254,long long _255,long long _256,bool _257);
static  bool issorted(std::shared_ptr< monty::ndarray< long long,1 > > _259,std::shared_ptr< monty::ndarray< int,1 > > _260,long long _261,long long _262,bool _263);
static  bool issorted(std::shared_ptr< monty::ndarray< long long,1 > > _265,std::shared_ptr< monty::ndarray< long long,1 > > _266,std::shared_ptr< monty::ndarray< long long,1 > > _267,long long _268,long long _269,bool _270);
static  bool issorted(std::shared_ptr< monty::ndarray< long long,1 > > _272,std::shared_ptr< monty::ndarray< int,1 > > _273,std::shared_ptr< monty::ndarray< int,1 > > _274,long long _275,long long _276,bool _277);
}; // struct Sort;

struct p_IndexCounter
{
IndexCounter * _pubthis;
static mosek::fusion::p_IndexCounter* _get_impl(mosek::fusion::IndexCounter * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_IndexCounter * _get_impl(mosek::fusion::IndexCounter::t _inst) { return _get_impl(_inst.get()); }
p_IndexCounter(IndexCounter * _pubthis);
virtual ~p_IndexCounter() { /* std::cout << "~p_IndexCounter" << std::endl;*/ };
long long start{};std::shared_ptr< monty::ndarray< int,1 > > dims{};std::shared_ptr< monty::ndarray< long long,1 > > strides{};std::shared_ptr< monty::ndarray< long long,1 > > st{};std::shared_ptr< monty::ndarray< int,1 > > ii{};int n{};virtual void destroy();
static IndexCounter::t _new_IndexCounter(std::shared_ptr< monty::ndarray< int,1 > > _279);
void _initialize(std::shared_ptr< monty::ndarray< int,1 > > _279);
static IndexCounter::t _new_IndexCounter(long long _281,std::shared_ptr< monty::ndarray< int,1 > > _282,std::shared_ptr< monty::ndarray< int,1 > > _283);
void _initialize(long long _281,std::shared_ptr< monty::ndarray< int,1 > > _282,std::shared_ptr< monty::ndarray< int,1 > > _283);
static IndexCounter::t _new_IndexCounter(long long _286,std::shared_ptr< monty::ndarray< int,1 > > _287,std::shared_ptr< monty::ndarray< long long,1 > > _288);
void _initialize(long long _286,std::shared_ptr< monty::ndarray< int,1 > > _287,std::shared_ptr< monty::ndarray< long long,1 > > _288);
virtual bool atEnd() ;
virtual std::shared_ptr< monty::ndarray< int,1 > > getIndex() ;
virtual long long next() ;
virtual long long get() ;
virtual void inc() ;
virtual void reset() ;
}; // struct IndexCounter;

struct p_CommonTools
{
CommonTools * _pubthis;
static mosek::fusion::p_CommonTools* _get_impl(mosek::fusion::CommonTools * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_CommonTools * _get_impl(mosek::fusion::CommonTools::t _inst) { return _get_impl(_inst.get()); }
p_CommonTools(CommonTools * _pubthis);
virtual ~p_CommonTools() { /* std::cout << "~p_CommonTools" << std::endl;*/ };
virtual void destroy();
static  std::shared_ptr< monty::ndarray< long long,1 > > resize(std::shared_ptr< monty::ndarray< long long,1 > > _294,int _295);
static  std::shared_ptr< monty::ndarray< int,1 > > resize(std::shared_ptr< monty::ndarray< int,1 > > _297,int _298);
static  std::shared_ptr< monty::ndarray< double,1 > > resize(std::shared_ptr< monty::ndarray< double,1 > > _300,int _301);
static  int binarySearch(std::shared_ptr< monty::ndarray< int,1 > > _303,int _304);
static  int binarySearch(std::shared_ptr< monty::ndarray< long long,1 > > _308,long long _309);
static  int binarySearchR(std::shared_ptr< monty::ndarray< long long,1 > > _311,long long _312);
static  int binarySearchL(std::shared_ptr< monty::ndarray< long long,1 > > _316,long long _317);
static  void ndIncr(std::shared_ptr< monty::ndarray< int,1 > > _321,std::shared_ptr< monty::ndarray< int,1 > > _322,std::shared_ptr< monty::ndarray< int,1 > > _323);
static  void transposeTriplets(std::shared_ptr< monty::ndarray< int,1 > > _325,std::shared_ptr< monty::ndarray< int,1 > > _326,std::shared_ptr< monty::ndarray< double,1 > > _327,std::shared_ptr< monty::ndarray< std::shared_ptr< monty::ndarray< long long,1 > >,1 > > _328,std::shared_ptr< monty::ndarray< std::shared_ptr< monty::ndarray< long long,1 > >,1 > > _329,std::shared_ptr< monty::ndarray< std::shared_ptr< monty::ndarray< double,1 > >,1 > > _330,long long _331,int _332,int _333);
static  void transposeTriplets(std::shared_ptr< monty::ndarray< int,1 > > _346,std::shared_ptr< monty::ndarray< int,1 > > _347,std::shared_ptr< monty::ndarray< double,1 > > _348,std::shared_ptr< monty::ndarray< std::shared_ptr< monty::ndarray< int,1 > >,1 > > _349,std::shared_ptr< monty::ndarray< std::shared_ptr< monty::ndarray< int,1 > >,1 > > _350,std::shared_ptr< monty::ndarray< std::shared_ptr< monty::ndarray< double,1 > >,1 > > _351,long long _352,int _353,int _354);
static  void tripletSort(std::shared_ptr< monty::ndarray< int,1 > > _367,std::shared_ptr< monty::ndarray< int,1 > > _368,std::shared_ptr< monty::ndarray< double,1 > > _369,std::shared_ptr< monty::ndarray< std::shared_ptr< monty::ndarray< int,1 > >,1 > > _370,std::shared_ptr< monty::ndarray< std::shared_ptr< monty::ndarray< int,1 > >,1 > > _371,std::shared_ptr< monty::ndarray< std::shared_ptr< monty::ndarray< double,1 > >,1 > > _372,long long _373,int _374,int _375);
static  void argMSort(std::shared_ptr< monty::ndarray< int,1 > > _401,std::shared_ptr< monty::ndarray< int,1 > > _402);
static  void mergeInto(std::shared_ptr< monty::ndarray< int,1 > > _407,std::shared_ptr< monty::ndarray< int,1 > > _408,std::shared_ptr< monty::ndarray< int,1 > > _409,int _410,int _411,int _412);
static  void argQsort(std::shared_ptr< monty::ndarray< long long,1 > > _418,std::shared_ptr< monty::ndarray< long long,1 > > _419,std::shared_ptr< monty::ndarray< long long,1 > > _420,long long _421,long long _422);
static  void argQsort(std::shared_ptr< monty::ndarray< long long,1 > > _423,std::shared_ptr< monty::ndarray< int,1 > > _424,std::shared_ptr< monty::ndarray< int,1 > > _425,long long _426,long long _427);
}; // struct CommonTools;

struct p_SolutionStruct
{
SolutionStruct * _pubthis;
static mosek::fusion::p_SolutionStruct* _get_impl(mosek::fusion::SolutionStruct * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_SolutionStruct * _get_impl(mosek::fusion::SolutionStruct::t _inst) { return _get_impl(_inst.get()); }
p_SolutionStruct(SolutionStruct * _pubthis);
virtual ~p_SolutionStruct() { /* std::cout << "~p_SolutionStruct" << std::endl;*/ };
std::shared_ptr< monty::ndarray< double,1 > > yx{};std::shared_ptr< monty::ndarray< double,1 > > snx{};std::shared_ptr< monty::ndarray< double,1 > > sux{};std::shared_ptr< monty::ndarray< double,1 > > slx{};std::shared_ptr< monty::ndarray< double,1 > > bars{};std::shared_ptr< monty::ndarray< double,1 > > barx{};std::shared_ptr< monty::ndarray< double,1 > > y{};std::shared_ptr< monty::ndarray< double,1 > > suc{};std::shared_ptr< monty::ndarray< double,1 > > slc{};std::shared_ptr< monty::ndarray< double,1 > > xx{};std::shared_ptr< monty::ndarray< double,1 > > xc{};double dobj{};double pobj{};mosek::fusion::ProblemStatus probstatus{};mosek::fusion::SolutionStatus dstatus{};mosek::fusion::SolutionStatus pstatus{};int sol_numbarvar{};int sol_numcone{};int sol_numvar{};int sol_numcon{};virtual void destroy();
static SolutionStruct::t _new_SolutionStruct(int _428,int _429,int _430,int _431);
void _initialize(int _428,int _429,int _430,int _431);
static SolutionStruct::t _new_SolutionStruct(monty::rc_ptr< ::mosek::fusion::SolutionStruct > _432);
void _initialize(monty::rc_ptr< ::mosek::fusion::SolutionStruct > _432);
virtual monty::rc_ptr< ::mosek::fusion::SolutionStruct > __mosek_2fusion_2SolutionStruct__clone() ;
virtual void resize(int _433,int _434,int _435,int _436) ;
virtual bool isDualAcceptable(mosek::fusion::AccSolutionStatus _456) ;
virtual bool isPrimalAcceptable(mosek::fusion::AccSolutionStatus _457) ;
virtual bool isAcceptable(mosek::fusion::SolutionStatus _458,mosek::fusion::AccSolutionStatus _459) ;
}; // struct SolutionStruct;

struct p_ConNZStruct
{
ConNZStruct * _pubthis;
static mosek::fusion::p_ConNZStruct* _get_impl(mosek::fusion::ConNZStruct * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_ConNZStruct * _get_impl(mosek::fusion::ConNZStruct::t _inst) { return _get_impl(_inst.get()); }
p_ConNZStruct(ConNZStruct * _pubthis);
virtual ~p_ConNZStruct() { /* std::cout << "~p_ConNZStruct" << std::endl;*/ };
std::shared_ptr< monty::ndarray< int,1 > > barmidx{};std::shared_ptr< monty::ndarray< int,1 > > barsubj{};std::shared_ptr< monty::ndarray< int,1 > > barsubi{};std::shared_ptr< monty::ndarray< double,1 > > bfix{};std::shared_ptr< monty::ndarray< double,1 > > cof{};std::shared_ptr< monty::ndarray< int,1 > > subj{};std::shared_ptr< monty::ndarray< long long,1 > > ptrb{};virtual void destroy();
static ConNZStruct::t _new_ConNZStruct(std::shared_ptr< monty::ndarray< long long,1 > > _460,std::shared_ptr< monty::ndarray< int,1 > > _461,std::shared_ptr< monty::ndarray< double,1 > > _462,std::shared_ptr< monty::ndarray< double,1 > > _463,std::shared_ptr< monty::ndarray< int,1 > > _464,std::shared_ptr< monty::ndarray< int,1 > > _465,std::shared_ptr< monty::ndarray< int,1 > > _466);
void _initialize(std::shared_ptr< monty::ndarray< long long,1 > > _460,std::shared_ptr< monty::ndarray< int,1 > > _461,std::shared_ptr< monty::ndarray< double,1 > > _462,std::shared_ptr< monty::ndarray< double,1 > > _463,std::shared_ptr< monty::ndarray< int,1 > > _464,std::shared_ptr< monty::ndarray< int,1 > > _465,std::shared_ptr< monty::ndarray< int,1 > > _466);
}; // struct ConNZStruct;

struct p_BaseVariable : public /*implements*/ virtual ::mosek::fusion::Variable
{
BaseVariable * _pubthis;
static mosek::fusion::p_BaseVariable* _get_impl(mosek::fusion::BaseVariable * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_BaseVariable * _get_impl(mosek::fusion::BaseVariable::t _inst) { return _get_impl(_inst.get()); }
p_BaseVariable(BaseVariable * _pubthis);
virtual ~p_BaseVariable() { /* std::cout << "~p_BaseVariable" << std::endl;*/ };
std::shared_ptr< monty::ndarray< long long,1 > > sparsity{};std::shared_ptr< monty::ndarray< long long,1 > > nativeidxs{};monty::rc_ptr< ::mosek::fusion::Model > model{};std::shared_ptr< monty::ndarray< int,1 > > shape{};virtual void destroy();
static BaseVariable::t _new_BaseVariable(monty::rc_ptr< ::mosek::fusion::BaseVariable > _1609,monty::rc_ptr< ::mosek::fusion::Model > _1610);
void _initialize(monty::rc_ptr< ::mosek::fusion::BaseVariable > _1609,monty::rc_ptr< ::mosek::fusion::Model > _1610);
static BaseVariable::t _new_BaseVariable(monty::rc_ptr< ::mosek::fusion::Model > _1611,std::shared_ptr< monty::ndarray< int,1 > > _1612,std::shared_ptr< monty::ndarray< long long,1 > > _1613,std::shared_ptr< monty::ndarray< long long,1 > > _1614);
void _initialize(monty::rc_ptr< ::mosek::fusion::Model > _1611,std::shared_ptr< monty::ndarray< int,1 > > _1612,std::shared_ptr< monty::ndarray< long long,1 > > _1613,std::shared_ptr< monty::ndarray< long long,1 > > _1614);
virtual /* override */ std::string toString() ;
virtual monty::rc_ptr< ::mosek::fusion::FlatExpr > __mosek_2fusion_2BaseVariable__eval() ;
virtual monty::rc_ptr< ::mosek::fusion::FlatExpr > __mosek_2fusion_2Expression__eval() { return __mosek_2fusion_2BaseVariable__eval(); }
virtual void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _1617,monty::rc_ptr< ::mosek::fusion::WorkStack > _1618,monty::rc_ptr< ::mosek::fusion::WorkStack > _1619) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__reshape(int _1639,int _1640,int _1641) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Variable__reshape(int _1639,int _1640,int _1641) { return __mosek_2fusion_2BaseVariable__reshape(_1639,_1640,_1641); }
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__reshape(int _1642,int _1643) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Variable__reshape(int _1642,int _1643) { return __mosek_2fusion_2BaseVariable__reshape(_1642,_1643); }
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__reshape(int _1644) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Variable__reshape(int _1644) { return __mosek_2fusion_2BaseVariable__reshape(_1644); }
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__reshape(std::shared_ptr< monty::ndarray< int,1 > > _1645) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Variable__reshape(std::shared_ptr< monty::ndarray< int,1 > > _1645) { return __mosek_2fusion_2BaseVariable__reshape(_1645); }
virtual void setLevel(std::shared_ptr< monty::ndarray< double,1 > > _1649) ;
virtual monty::rc_ptr< ::mosek::fusion::Model > __mosek_2fusion_2BaseVariable__getModel() ;
virtual monty::rc_ptr< ::mosek::fusion::Model > __mosek_2fusion_2Variable__getModel() { return __mosek_2fusion_2BaseVariable__getModel(); }
virtual int getND() ;
virtual int getDim(int _1652) ;
virtual std::shared_ptr< monty::ndarray< int,1 > > getShape() ;
virtual long long getSize() ;
virtual std::shared_ptr< monty::ndarray< double,1 > > dual() ;
virtual std::shared_ptr< monty::ndarray< double,1 > > level() ;
virtual void makeContinuous() ;
virtual void makeInteger() ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__transpose() ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Variable__transpose() { return __mosek_2fusion_2BaseVariable__transpose(); }
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__index(int _1673,int _1674,int _1675) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Variable__index(int _1673,int _1674,int _1675) { return __mosek_2fusion_2BaseVariable__index(_1673,_1674,_1675); }
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__index(int _1676,int _1677) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Variable__index(int _1676,int _1677) { return __mosek_2fusion_2BaseVariable__index(_1676,_1677); }
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__index(std::shared_ptr< monty::ndarray< int,1 > > _1678) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Variable__index(std::shared_ptr< monty::ndarray< int,1 > > _1678) { return __mosek_2fusion_2BaseVariable__index(_1678); }
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__index(int _1681) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Variable__index(int _1681) { return __mosek_2fusion_2BaseVariable__index(_1681); }
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__pick(std::shared_ptr< monty::ndarray< int,1 > > _1682,std::shared_ptr< monty::ndarray< int,1 > > _1683,std::shared_ptr< monty::ndarray< int,1 > > _1684) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Variable__pick(std::shared_ptr< monty::ndarray< int,1 > > _1682,std::shared_ptr< monty::ndarray< int,1 > > _1683,std::shared_ptr< monty::ndarray< int,1 > > _1684) { return __mosek_2fusion_2BaseVariable__pick(_1682,_1683,_1684); }
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__pick(std::shared_ptr< monty::ndarray< int,1 > > _1687,std::shared_ptr< monty::ndarray< int,1 > > _1688) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Variable__pick(std::shared_ptr< monty::ndarray< int,1 > > _1687,std::shared_ptr< monty::ndarray< int,1 > > _1688) { return __mosek_2fusion_2BaseVariable__pick(_1687,_1688); }
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__pick(std::shared_ptr< monty::ndarray< int,2 > > _1691) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Variable__pick(std::shared_ptr< monty::ndarray< int,2 > > _1691) { return __mosek_2fusion_2BaseVariable__pick(_1691); }
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__pick(std::shared_ptr< monty::ndarray< int,1 > > _1713) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Variable__pick(std::shared_ptr< monty::ndarray< int,1 > > _1713) { return __mosek_2fusion_2BaseVariable__pick(_1713); }
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__antidiag(int _1724) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Variable__antidiag(int _1724) { return __mosek_2fusion_2BaseVariable__antidiag(_1724); }
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__antidiag() ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Variable__antidiag() { return __mosek_2fusion_2BaseVariable__antidiag(); }
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__diag(int _1725) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Variable__diag(int _1725) { return __mosek_2fusion_2BaseVariable__diag(_1725); }
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__diag() ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Variable__diag() { return __mosek_2fusion_2BaseVariable__diag(); }
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__general_diag(std::shared_ptr< monty::ndarray< int,1 > > _1726,std::shared_ptr< monty::ndarray< int,1 > > _1727,int _1728) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__slice(std::shared_ptr< monty::ndarray< int,1 > > _1749,std::shared_ptr< monty::ndarray< int,1 > > _1750) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Variable__slice(std::shared_ptr< monty::ndarray< int,1 > > _1749,std::shared_ptr< monty::ndarray< int,1 > > _1750) { return __mosek_2fusion_2BaseVariable__slice(_1749,_1750); }
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__slice(int _1784,int _1785) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Variable__slice(int _1784,int _1785) { return __mosek_2fusion_2BaseVariable__slice(_1784,_1785); }
virtual monty::rc_ptr< ::mosek::fusion::Expression > __mosek_2fusion_2BaseVariable__asExpr() ;
virtual monty::rc_ptr< ::mosek::fusion::Expression > __mosek_2fusion_2Variable__asExpr() { return __mosek_2fusion_2BaseVariable__asExpr(); }
virtual int inst(int _1794,std::shared_ptr< monty::ndarray< long long,1 > > _1795,int _1796,std::shared_ptr< monty::ndarray< long long,1 > > _1797) ;
virtual int numInst() ;
virtual void inst(int _1802,std::shared_ptr< monty::ndarray< long long,1 > > _1803) ;
virtual void set_values(std::shared_ptr< monty::ndarray< double,1 > > _1810,bool _1811) ;
virtual void dual_lu(int _1816,std::shared_ptr< monty::ndarray< double,1 > > _1817,bool _1818) ;
virtual void values(int _1821,std::shared_ptr< monty::ndarray< double,1 > > _1822,bool _1823) ;
virtual void make_continuous() ;
virtual void make_integer() ;
}; // struct BaseVariable;

struct p_SliceVariable : public ::mosek::fusion::p_BaseVariable
{
SliceVariable * _pubthis;
static mosek::fusion::p_SliceVariable* _get_impl(mosek::fusion::SliceVariable * _inst){ return static_cast< mosek::fusion::p_SliceVariable* >(mosek::fusion::p_BaseVariable::_get_impl(_inst)); }
static mosek::fusion::p_SliceVariable * _get_impl(mosek::fusion::SliceVariable::t _inst) { return _get_impl(_inst.get()); }
p_SliceVariable(SliceVariable * _pubthis);
virtual ~p_SliceVariable() { /* std::cout << "~p_SliceVariable" << std::endl;*/ };
std::shared_ptr< monty::ndarray< int,1 > > shape{};std::shared_ptr< monty::ndarray< long long,1 > > sparsity{};std::shared_ptr< monty::ndarray< long long,1 > > nativeidxs{};virtual void destroy();
static SliceVariable::t _new_SliceVariable(monty::rc_ptr< ::mosek::fusion::Model > _1369,std::shared_ptr< monty::ndarray< int,1 > > _1370,std::shared_ptr< monty::ndarray< long long,1 > > _1371,std::shared_ptr< monty::ndarray< long long,1 > > _1372);
void _initialize(monty::rc_ptr< ::mosek::fusion::Model > _1369,std::shared_ptr< monty::ndarray< int,1 > > _1370,std::shared_ptr< monty::ndarray< long long,1 > > _1371,std::shared_ptr< monty::ndarray< long long,1 > > _1372);
}; // struct SliceVariable;

struct p_ModelVariable : public ::mosek::fusion::p_BaseVariable
{
ModelVariable * _pubthis;
static mosek::fusion::p_ModelVariable* _get_impl(mosek::fusion::ModelVariable * _inst){ return static_cast< mosek::fusion::p_ModelVariable* >(mosek::fusion::p_BaseVariable::_get_impl(_inst)); }
static mosek::fusion::p_ModelVariable * _get_impl(mosek::fusion::ModelVariable::t _inst) { return _get_impl(_inst.get()); }
p_ModelVariable(ModelVariable * _pubthis);
virtual ~p_ModelVariable() { /* std::cout << "~p_ModelVariable" << std::endl;*/ };
std::shared_ptr< monty::ndarray< long long,1 > > sparsity{};std::shared_ptr< monty::ndarray< int,1 > > shape{};std::shared_ptr< monty::ndarray< long long,1 > > nativeidxs{};long long varid{};std::string name{};virtual void destroy();
static ModelVariable::t _new_ModelVariable(monty::rc_ptr< ::mosek::fusion::ModelVariable > _1572,monty::rc_ptr< ::mosek::fusion::Model > _1573);
void _initialize(monty::rc_ptr< ::mosek::fusion::ModelVariable > _1572,monty::rc_ptr< ::mosek::fusion::Model > _1573);
static ModelVariable::t _new_ModelVariable(monty::rc_ptr< ::mosek::fusion::Model > _1574,const std::string &  _1575,std::shared_ptr< monty::ndarray< int,1 > > _1576,long long _1577,std::shared_ptr< monty::ndarray< long long,1 > > _1578,std::shared_ptr< monty::ndarray< long long,1 > > _1579);
void _initialize(monty::rc_ptr< ::mosek::fusion::Model > _1574,const std::string &  _1575,std::shared_ptr< monty::ndarray< int,1 > > _1576,long long _1577,std::shared_ptr< monty::ndarray< long long,1 > > _1578,std::shared_ptr< monty::ndarray< long long,1 > > _1579);
virtual void flushNames() { throw monty::AbstractClassError("Call to abstract method"); }
virtual void elementName(long long _1580,monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > _1581) ;
virtual monty::rc_ptr< ::mosek::fusion::ModelVariable > __mosek_2fusion_2ModelVariable__clone(monty::rc_ptr< ::mosek::fusion::Model > _1582) { throw monty::AbstractClassError("Call to abstract method"); }
}; // struct ModelVariable;

struct p_SymRangedVariable : public ::mosek::fusion::p_ModelVariable, public /*implements*/ virtual ::mosek::fusion::SymmetricVariable
{
SymRangedVariable * _pubthis;
static mosek::fusion::p_SymRangedVariable* _get_impl(mosek::fusion::SymRangedVariable * _inst){ return static_cast< mosek::fusion::p_SymRangedVariable* >(mosek::fusion::p_ModelVariable::_get_impl(_inst)); }
static mosek::fusion::p_SymRangedVariable * _get_impl(mosek::fusion::SymRangedVariable::t _inst) { return _get_impl(_inst.get()); }
p_SymRangedVariable(SymRangedVariable * _pubthis);
virtual ~p_SymRangedVariable() { /* std::cout << "~p_SymRangedVariable" << std::endl;*/ };
int dim{};std::shared_ptr< monty::ndarray< long long,1 > > sparsity{};std::shared_ptr< monty::ndarray< int,1 > > nativeidxs{};bool names_flushed{};std::string name{};virtual void destroy();
static SymRangedVariable::t _new_SymRangedVariable(monty::rc_ptr< ::mosek::fusion::SymRangedVariable > _1373,monty::rc_ptr< ::mosek::fusion::Model > _1374);
void _initialize(monty::rc_ptr< ::mosek::fusion::SymRangedVariable > _1373,monty::rc_ptr< ::mosek::fusion::Model > _1374);
static SymRangedVariable::t _new_SymRangedVariable(monty::rc_ptr< ::mosek::fusion::Model > _1375,const std::string &  _1376,long long _1377,int _1378,std::shared_ptr< monty::ndarray< long long,1 > > _1379,std::shared_ptr< monty::ndarray< int,1 > > _1380);
void _initialize(monty::rc_ptr< ::mosek::fusion::Model > _1375,const std::string &  _1376,long long _1377,int _1378,std::shared_ptr< monty::ndarray< long long,1 > > _1379,std::shared_ptr< monty::ndarray< int,1 > > _1380);
virtual void dual_u(int _1381,std::shared_ptr< monty::ndarray< double,1 > > _1382) ;
virtual void dual_l(int _1383,std::shared_ptr< monty::ndarray< double,1 > > _1384) ;
virtual /* override */ void flushNames() ;
virtual /* override */ monty::rc_ptr< ::mosek::fusion::ModelVariable > __mosek_2fusion_2SymRangedVariable__clone(monty::rc_ptr< ::mosek::fusion::Model > _1388) ;
virtual monty::rc_ptr< ::mosek::fusion::ModelVariable > __mosek_2fusion_2ModelVariable__clone(monty::rc_ptr< ::mosek::fusion::Model > _1388) { return __mosek_2fusion_2SymRangedVariable__clone(_1388); }
static  std::shared_ptr< monty::ndarray< long long,1 > > mirror_idxs(int _1389,std::shared_ptr< monty::ndarray< long long,1 > > _1390,std::shared_ptr< monty::ndarray< int,1 > > _1391);
static  std::shared_ptr< monty::ndarray< long long,1 > > mirror_sp(int _1407,std::shared_ptr< monty::ndarray< long long,1 > > _1408);
}; // struct SymRangedVariable;

struct p_RangedVariable : public ::mosek::fusion::p_ModelVariable
{
RangedVariable * _pubthis;
static mosek::fusion::p_RangedVariable* _get_impl(mosek::fusion::RangedVariable * _inst){ return static_cast< mosek::fusion::p_RangedVariable* >(mosek::fusion::p_ModelVariable::_get_impl(_inst)); }
static mosek::fusion::p_RangedVariable * _get_impl(mosek::fusion::RangedVariable::t _inst) { return _get_impl(_inst.get()); }
p_RangedVariable(RangedVariable * _pubthis);
virtual ~p_RangedVariable() { /* std::cout << "~p_RangedVariable" << std::endl;*/ };
std::shared_ptr< monty::ndarray< int,1 > > shape{};std::string name{};bool names_flushed{};std::shared_ptr< monty::ndarray< int,1 > > nativeidxs{};std::shared_ptr< monty::ndarray< long long,1 > > sparsity{};virtual void destroy();
static RangedVariable::t _new_RangedVariable(monty::rc_ptr< ::mosek::fusion::RangedVariable > _1419,monty::rc_ptr< ::mosek::fusion::Model > _1420);
void _initialize(monty::rc_ptr< ::mosek::fusion::RangedVariable > _1419,monty::rc_ptr< ::mosek::fusion::Model > _1420);
static RangedVariable::t _new_RangedVariable(monty::rc_ptr< ::mosek::fusion::Model > _1421,const std::string &  _1422,long long _1423,std::shared_ptr< monty::ndarray< int,1 > > _1424,std::shared_ptr< monty::ndarray< long long,1 > > _1425,std::shared_ptr< monty::ndarray< int,1 > > _1426);
void _initialize(monty::rc_ptr< ::mosek::fusion::Model > _1421,const std::string &  _1422,long long _1423,std::shared_ptr< monty::ndarray< int,1 > > _1424,std::shared_ptr< monty::ndarray< long long,1 > > _1425,std::shared_ptr< monty::ndarray< int,1 > > _1426);
virtual monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > __mosek_2fusion_2RangedVariable__elementDesc(long long _1427,monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > _1428) ;
virtual /* override */ void flushNames() ;
virtual void dual_u(int _1432,std::shared_ptr< monty::ndarray< double,1 > > _1433) ;
virtual void dual_l(int _1434,std::shared_ptr< monty::ndarray< double,1 > > _1435) ;
virtual /* override */ monty::rc_ptr< ::mosek::fusion::ModelVariable > __mosek_2fusion_2RangedVariable__clone(monty::rc_ptr< ::mosek::fusion::Model > _1436) ;
virtual monty::rc_ptr< ::mosek::fusion::ModelVariable > __mosek_2fusion_2ModelVariable__clone(monty::rc_ptr< ::mosek::fusion::Model > _1436) { return __mosek_2fusion_2RangedVariable__clone(_1436); }
static  std::shared_ptr< monty::ndarray< long long,1 > > globalNativeIndexes(std::shared_ptr< monty::ndarray< int,1 > > _1437);
}; // struct RangedVariable;

struct p_LinearPSDVariable : public ::mosek::fusion::p_ModelVariable
{
LinearPSDVariable * _pubthis;
static mosek::fusion::p_LinearPSDVariable* _get_impl(mosek::fusion::LinearPSDVariable * _inst){ return static_cast< mosek::fusion::p_LinearPSDVariable* >(mosek::fusion::p_ModelVariable::_get_impl(_inst)); }
static mosek::fusion::p_LinearPSDVariable * _get_impl(mosek::fusion::LinearPSDVariable::t _inst) { return _get_impl(_inst.get()); }
p_LinearPSDVariable(LinearPSDVariable * _pubthis);
virtual ~p_LinearPSDVariable() { /* std::cout << "~p_LinearPSDVariable" << std::endl;*/ };
std::shared_ptr< monty::ndarray< int,1 > > shape{};std::string name{};int varid{};std::shared_ptr< monty::ndarray< long long,1 > > nativeidxs{};int conedim{};virtual void destroy();
static LinearPSDVariable::t _new_LinearPSDVariable(monty::rc_ptr< ::mosek::fusion::LinearPSDVariable > _1440,monty::rc_ptr< ::mosek::fusion::Model > _1441);
void _initialize(monty::rc_ptr< ::mosek::fusion::LinearPSDVariable > _1440,monty::rc_ptr< ::mosek::fusion::Model > _1441);
static LinearPSDVariable::t _new_LinearPSDVariable(monty::rc_ptr< ::mosek::fusion::Model > _1442,const std::string &  _1443,int _1444,std::shared_ptr< monty::ndarray< int,1 > > _1445,int _1446,std::shared_ptr< monty::ndarray< long long,1 > > _1447);
void _initialize(monty::rc_ptr< ::mosek::fusion::Model > _1442,const std::string &  _1443,int _1444,std::shared_ptr< monty::ndarray< int,1 > > _1445,int _1446,std::shared_ptr< monty::ndarray< long long,1 > > _1447);
virtual /* override */ void flushNames() ;
virtual /* override */ std::string toString() ;
virtual void make_continuous(std::shared_ptr< monty::ndarray< long long,1 > > _1450) ;
virtual void make_integer(std::shared_ptr< monty::ndarray< long long,1 > > _1451) ;
virtual /* override */ monty::rc_ptr< ::mosek::fusion::ModelVariable > __mosek_2fusion_2LinearPSDVariable__clone(monty::rc_ptr< ::mosek::fusion::Model > _1452) ;
virtual monty::rc_ptr< ::mosek::fusion::ModelVariable > __mosek_2fusion_2ModelVariable__clone(monty::rc_ptr< ::mosek::fusion::Model > _1452) { return __mosek_2fusion_2LinearPSDVariable__clone(_1452); }
static  std::shared_ptr< monty::ndarray< long long,1 > > globalNativeIndexes(std::shared_ptr< monty::ndarray< long long,1 > > _1453);
}; // struct LinearPSDVariable;

struct p_PSDVariable : public ::mosek::fusion::p_ModelVariable, public /*implements*/ virtual ::mosek::fusion::SymmetricVariable
{
PSDVariable * _pubthis;
static mosek::fusion::p_PSDVariable* _get_impl(mosek::fusion::PSDVariable * _inst){ return static_cast< mosek::fusion::p_PSDVariable* >(mosek::fusion::p_ModelVariable::_get_impl(_inst)); }
static mosek::fusion::p_PSDVariable * _get_impl(mosek::fusion::PSDVariable::t _inst) { return _get_impl(_inst.get()); }
p_PSDVariable(PSDVariable * _pubthis);
virtual ~p_PSDVariable() { /* std::cout << "~p_PSDVariable" << std::endl;*/ };
int conedim2{};int conedim1{};std::shared_ptr< monty::ndarray< int,1 > > shape{};std::string name{};std::shared_ptr< monty::ndarray< long long,1 > > nativeidxs{};int varid{};virtual void destroy();
static PSDVariable::t _new_PSDVariable(monty::rc_ptr< ::mosek::fusion::PSDVariable > _1455,monty::rc_ptr< ::mosek::fusion::Model > _1456);
void _initialize(monty::rc_ptr< ::mosek::fusion::PSDVariable > _1455,monty::rc_ptr< ::mosek::fusion::Model > _1456);
static PSDVariable::t _new_PSDVariable(monty::rc_ptr< ::mosek::fusion::Model > _1457,const std::string &  _1458,int _1459,std::shared_ptr< monty::ndarray< int,1 > > _1460,int _1461,int _1462,std::shared_ptr< monty::ndarray< long long,1 > > _1463);
void _initialize(monty::rc_ptr< ::mosek::fusion::Model > _1457,const std::string &  _1458,int _1459,std::shared_ptr< monty::ndarray< int,1 > > _1460,int _1461,int _1462,std::shared_ptr< monty::ndarray< long long,1 > > _1463);
virtual /* override */ void flushNames() ;
virtual /* override */ std::string toString() ;
virtual monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > __mosek_2fusion_2PSDVariable__elementDesc(long long _1466,monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > _1467) ;
virtual /* override */ monty::rc_ptr< ::mosek::fusion::ModelVariable > __mosek_2fusion_2PSDVariable__clone(monty::rc_ptr< ::mosek::fusion::Model > _1468) ;
virtual monty::rc_ptr< ::mosek::fusion::ModelVariable > __mosek_2fusion_2ModelVariable__clone(monty::rc_ptr< ::mosek::fusion::Model > _1468) { return __mosek_2fusion_2PSDVariable__clone(_1468); }
static  std::shared_ptr< monty::ndarray< long long,1 > > fullnativeidxs(std::shared_ptr< monty::ndarray< int,1 > > _1469,int _1470,int _1471,std::shared_ptr< monty::ndarray< long long,1 > > _1472);
}; // struct PSDVariable;

struct p_SymLinearVariable : public ::mosek::fusion::p_ModelVariable, public /*implements*/ virtual ::mosek::fusion::SymmetricVariable
{
SymLinearVariable * _pubthis;
static mosek::fusion::p_SymLinearVariable* _get_impl(mosek::fusion::SymLinearVariable * _inst){ return static_cast< mosek::fusion::p_SymLinearVariable* >(mosek::fusion::p_ModelVariable::_get_impl(_inst)); }
static mosek::fusion::p_SymLinearVariable * _get_impl(mosek::fusion::SymLinearVariable::t _inst) { return _get_impl(_inst.get()); }
p_SymLinearVariable(SymLinearVariable * _pubthis);
virtual ~p_SymLinearVariable() { /* std::cout << "~p_SymLinearVariable" << std::endl;*/ };
int dim{};std::shared_ptr< monty::ndarray< long long,1 > > sparsity{};std::shared_ptr< monty::ndarray< int,1 > > nativeidxs{};bool names_flushed{};std::string name{};virtual void destroy();
static SymLinearVariable::t _new_SymLinearVariable(monty::rc_ptr< ::mosek::fusion::SymLinearVariable > _1497,monty::rc_ptr< ::mosek::fusion::Model > _1498);
void _initialize(monty::rc_ptr< ::mosek::fusion::SymLinearVariable > _1497,monty::rc_ptr< ::mosek::fusion::Model > _1498);
static SymLinearVariable::t _new_SymLinearVariable(monty::rc_ptr< ::mosek::fusion::Model > _1499,const std::string &  _1500,long long _1501,int _1502,std::shared_ptr< monty::ndarray< long long,1 > > _1503,std::shared_ptr< monty::ndarray< int,1 > > _1504);
void _initialize(monty::rc_ptr< ::mosek::fusion::Model > _1499,const std::string &  _1500,long long _1501,int _1502,std::shared_ptr< monty::ndarray< long long,1 > > _1503,std::shared_ptr< monty::ndarray< int,1 > > _1504);
virtual /* override */ void flushNames() ;
virtual /* override */ monty::rc_ptr< ::mosek::fusion::ModelVariable > __mosek_2fusion_2SymLinearVariable__clone(monty::rc_ptr< ::mosek::fusion::Model > _1508) ;
virtual monty::rc_ptr< ::mosek::fusion::ModelVariable > __mosek_2fusion_2ModelVariable__clone(monty::rc_ptr< ::mosek::fusion::Model > _1508) { return __mosek_2fusion_2SymLinearVariable__clone(_1508); }
static  std::shared_ptr< monty::ndarray< long long,1 > > mirror_idxs(int _1509,std::shared_ptr< monty::ndarray< long long,1 > > _1510,std::shared_ptr< monty::ndarray< int,1 > > _1511);
static  std::shared_ptr< monty::ndarray< long long,1 > > mirror_sp(int _1527,std::shared_ptr< monty::ndarray< long long,1 > > _1528);
}; // struct SymLinearVariable;

struct p_LinearVariable : public ::mosek::fusion::p_ModelVariable
{
LinearVariable * _pubthis;
static mosek::fusion::p_LinearVariable* _get_impl(mosek::fusion::LinearVariable * _inst){ return static_cast< mosek::fusion::p_LinearVariable* >(mosek::fusion::p_ModelVariable::_get_impl(_inst)); }
static mosek::fusion::p_LinearVariable * _get_impl(mosek::fusion::LinearVariable::t _inst) { return _get_impl(_inst.get()); }
p_LinearVariable(LinearVariable * _pubthis);
virtual ~p_LinearVariable() { /* std::cout << "~p_LinearVariable" << std::endl;*/ };
std::shared_ptr< monty::ndarray< int,1 > > shape{};std::shared_ptr< monty::ndarray< long long,1 > > sparsity{};std::shared_ptr< monty::ndarray< int,1 > > nativeidxs{};bool names_flushed{};std::string name{};virtual void destroy();
static LinearVariable::t _new_LinearVariable(monty::rc_ptr< ::mosek::fusion::LinearVariable > _1539,monty::rc_ptr< ::mosek::fusion::Model > _1540);
void _initialize(monty::rc_ptr< ::mosek::fusion::LinearVariable > _1539,monty::rc_ptr< ::mosek::fusion::Model > _1540);
static LinearVariable::t _new_LinearVariable(monty::rc_ptr< ::mosek::fusion::Model > _1541,const std::string &  _1542,long long _1543,std::shared_ptr< monty::ndarray< int,1 > > _1544,std::shared_ptr< monty::ndarray< long long,1 > > _1545,std::shared_ptr< monty::ndarray< int,1 > > _1546);
void _initialize(monty::rc_ptr< ::mosek::fusion::Model > _1541,const std::string &  _1542,long long _1543,std::shared_ptr< monty::ndarray< int,1 > > _1544,std::shared_ptr< monty::ndarray< long long,1 > > _1545,std::shared_ptr< monty::ndarray< int,1 > > _1546);
virtual /* override */ std::string toString() ;
virtual /* override */ void flushNames() ;
virtual /* override */ monty::rc_ptr< ::mosek::fusion::ModelVariable > __mosek_2fusion_2LinearVariable__clone(monty::rc_ptr< ::mosek::fusion::Model > _1552) ;
virtual monty::rc_ptr< ::mosek::fusion::ModelVariable > __mosek_2fusion_2ModelVariable__clone(monty::rc_ptr< ::mosek::fusion::Model > _1552) { return __mosek_2fusion_2LinearVariable__clone(_1552); }
static  std::shared_ptr< monty::ndarray< long long,1 > > globalNativeIndexes(std::shared_ptr< monty::ndarray< int,1 > > _1553);
}; // struct LinearVariable;

struct p_ConicVariable : public ::mosek::fusion::p_ModelVariable
{
ConicVariable * _pubthis;
static mosek::fusion::p_ConicVariable* _get_impl(mosek::fusion::ConicVariable * _inst){ return static_cast< mosek::fusion::p_ConicVariable* >(mosek::fusion::p_ModelVariable::_get_impl(_inst)); }
static mosek::fusion::p_ConicVariable * _get_impl(mosek::fusion::ConicVariable::t _inst) { return _get_impl(_inst.get()); }
p_ConicVariable(ConicVariable * _pubthis);
virtual ~p_ConicVariable() { /* std::cout << "~p_ConicVariable" << std::endl;*/ };
std::shared_ptr< monty::ndarray< int,1 > > nativeidxs{};std::shared_ptr< monty::ndarray< int,1 > > shape{};std::string name{};bool names_flushed{};int varid{};virtual void destroy();
static ConicVariable::t _new_ConicVariable(monty::rc_ptr< ::mosek::fusion::ConicVariable > _1556,monty::rc_ptr< ::mosek::fusion::Model > _1557);
void _initialize(monty::rc_ptr< ::mosek::fusion::ConicVariable > _1556,monty::rc_ptr< ::mosek::fusion::Model > _1557);
static ConicVariable::t _new_ConicVariable(monty::rc_ptr< ::mosek::fusion::Model > _1558,const std::string &  _1559,int _1560,std::shared_ptr< monty::ndarray< int,1 > > _1561,std::shared_ptr< monty::ndarray< int,1 > > _1562);
void _initialize(monty::rc_ptr< ::mosek::fusion::Model > _1558,const std::string &  _1559,int _1560,std::shared_ptr< monty::ndarray< int,1 > > _1561,std::shared_ptr< monty::ndarray< int,1 > > _1562);
virtual /* override */ std::string toString() ;
virtual /* override */ void flushNames() ;
virtual /* override */ monty::rc_ptr< ::mosek::fusion::ModelVariable > __mosek_2fusion_2ConicVariable__clone(monty::rc_ptr< ::mosek::fusion::Model > _1568) ;
virtual monty::rc_ptr< ::mosek::fusion::ModelVariable > __mosek_2fusion_2ModelVariable__clone(monty::rc_ptr< ::mosek::fusion::Model > _1568) { return __mosek_2fusion_2ConicVariable__clone(_1568); }
static  std::shared_ptr< monty::ndarray< long long,1 > > globalNativeIndexes(std::shared_ptr< monty::ndarray< int,1 > > _1569);
}; // struct ConicVariable;

struct p_NilVariable : public ::mosek::fusion::p_BaseVariable, public /*implements*/ virtual ::mosek::fusion::SymmetricVariable
{
NilVariable * _pubthis;
static mosek::fusion::p_NilVariable* _get_impl(mosek::fusion::NilVariable * _inst){ return static_cast< mosek::fusion::p_NilVariable* >(mosek::fusion::p_BaseVariable::_get_impl(_inst)); }
static mosek::fusion::p_NilVariable * _get_impl(mosek::fusion::NilVariable::t _inst) { return _get_impl(_inst.get()); }
p_NilVariable(NilVariable * _pubthis);
virtual ~p_NilVariable() { /* std::cout << "~p_NilVariable" << std::endl;*/ };
std::shared_ptr< monty::ndarray< int,1 > > shape{};virtual void destroy();
static NilVariable::t _new_NilVariable(std::shared_ptr< monty::ndarray< int,1 > > _1583);
void _initialize(std::shared_ptr< monty::ndarray< int,1 > > _1583);
static NilVariable::t _new_NilVariable();
void _initialize();
virtual void flushNames() ;
virtual monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > __mosek_2fusion_2NilVariable__elementDesc(long long _1585,monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > _1586) ;
virtual void elementName(long long _1587,monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > _1588) ;
virtual /* override */ int numInst() ;
virtual int inst(int _1589,std::shared_ptr< monty::ndarray< long long,1 > > _1590,std::shared_ptr< monty::ndarray< long long,1 > > _1591) ;
virtual /* override */ void inst(int _1592,std::shared_ptr< monty::ndarray< long long,1 > > _1593) ;
virtual /* override */ void set_values(std::shared_ptr< monty::ndarray< double,1 > > _1594,bool _1595) ;
virtual /* override */ void values(int _1596,std::shared_ptr< monty::ndarray< double,1 > > _1597,bool _1598) ;
virtual /* override */ void make_continuous() ;
virtual /* override */ void make_integer() ;
virtual /* override */ std::string toString() ;
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2NilVariable__index(std::shared_ptr< monty::ndarray< int,1 > > _1599) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__index(std::shared_ptr< monty::ndarray< int,1 > > _1599) { return __mosek_2fusion_2NilVariable__index(_1599); }
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Variable__index(std::shared_ptr< monty::ndarray< int,1 > > _1599) { return __mosek_2fusion_2NilVariable__index(_1599); }
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2NilVariable__index(int _1601) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__index(int _1601) { return __mosek_2fusion_2NilVariable__index(_1601); }
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Variable__index(int _1601) { return __mosek_2fusion_2NilVariable__index(_1601); }
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2NilVariable__slice(std::shared_ptr< monty::ndarray< int,1 > > _1603,std::shared_ptr< monty::ndarray< int,1 > > _1604) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__slice(std::shared_ptr< monty::ndarray< int,1 > > _1603,std::shared_ptr< monty::ndarray< int,1 > > _1604) { return __mosek_2fusion_2NilVariable__slice(_1603,_1604); }
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Variable__slice(std::shared_ptr< monty::ndarray< int,1 > > _1603,std::shared_ptr< monty::ndarray< int,1 > > _1604) { return __mosek_2fusion_2NilVariable__slice(_1603,_1604); }
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2NilVariable__slice(int _1607,int _1608) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__slice(int _1607,int _1608) { return __mosek_2fusion_2NilVariable__slice(_1607,_1608); }
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Variable__slice(int _1607,int _1608) { return __mosek_2fusion_2NilVariable__slice(_1607,_1608); }
}; // struct NilVariable;

struct p_Var
{
Var * _pubthis;
static mosek::fusion::p_Var* _get_impl(mosek::fusion::Var * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_Var * _get_impl(mosek::fusion::Var::t _inst) { return _get_impl(_inst.get()); }
p_Var(Var * _pubthis);
virtual ~p_Var() { /* std::cout << "~p_Var" << std::endl;*/ };
virtual void destroy();
static  monty::rc_ptr< ::mosek::fusion::Variable > empty(std::shared_ptr< monty::ndarray< int,1 > > _1865);
static  monty::rc_ptr< ::mosek::fusion::Variable > compress(monty::rc_ptr< ::mosek::fusion::Variable > _1867);
static  monty::rc_ptr< ::mosek::fusion::Variable > reshape(monty::rc_ptr< ::mosek::fusion::Variable > _1875,int _1876);
static  monty::rc_ptr< ::mosek::fusion::Variable > reshape(monty::rc_ptr< ::mosek::fusion::Variable > _1877,int _1878,int _1879);
static  monty::rc_ptr< ::mosek::fusion::Variable > flatten(monty::rc_ptr< ::mosek::fusion::Variable > _1880);
static  monty::rc_ptr< ::mosek::fusion::Variable > reshape(monty::rc_ptr< ::mosek::fusion::Variable > _1881,std::shared_ptr< monty::ndarray< int,1 > > _1882);
static  monty::rc_ptr< ::mosek::fusion::Variable > index_permute_(monty::rc_ptr< ::mosek::fusion::Variable > _1883,std::shared_ptr< monty::ndarray< int,1 > > _1884);
static  monty::rc_ptr< ::mosek::fusion::Variable > hrepeat(monty::rc_ptr< ::mosek::fusion::Variable > _1913,int _1914);
static  monty::rc_ptr< ::mosek::fusion::Variable > vrepeat(monty::rc_ptr< ::mosek::fusion::Variable > _1915,int _1916);
static  monty::rc_ptr< ::mosek::fusion::Variable > repeat(monty::rc_ptr< ::mosek::fusion::Variable > _1917,int _1918);
static  monty::rc_ptr< ::mosek::fusion::Variable > repeat(monty::rc_ptr< ::mosek::fusion::Variable > _1919,int _1920,int _1921);
static  monty::rc_ptr< ::mosek::fusion::Variable > drepeat(monty::rc_ptr< ::mosek::fusion::Variable > _1922,int _1923,int _1924);
static  monty::rc_ptr< ::mosek::fusion::Variable > stack(std::shared_ptr< monty::ndarray< std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Variable >,1 > >,1 > > _1988);
static  monty::rc_ptr< ::mosek::fusion::Variable > vstack(monty::rc_ptr< ::mosek::fusion::Variable > _1990,monty::rc_ptr< ::mosek::fusion::Variable > _1991,monty::rc_ptr< ::mosek::fusion::Variable > _1992);
static  monty::rc_ptr< ::mosek::fusion::Variable > vstack(monty::rc_ptr< ::mosek::fusion::Variable > _1993,monty::rc_ptr< ::mosek::fusion::Variable > _1994);
static  monty::rc_ptr< ::mosek::fusion::Variable > vstack(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Variable >,1 > > _1995);
static  monty::rc_ptr< ::mosek::fusion::Variable > hstack(monty::rc_ptr< ::mosek::fusion::Variable > _1996,monty::rc_ptr< ::mosek::fusion::Variable > _1997,monty::rc_ptr< ::mosek::fusion::Variable > _1998);
static  monty::rc_ptr< ::mosek::fusion::Variable > hstack(monty::rc_ptr< ::mosek::fusion::Variable > _1999,monty::rc_ptr< ::mosek::fusion::Variable > _2000);
static  monty::rc_ptr< ::mosek::fusion::Variable > hstack(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Variable >,1 > > _2001);
static  monty::rc_ptr< ::mosek::fusion::Variable > stack(monty::rc_ptr< ::mosek::fusion::Variable > _2002,monty::rc_ptr< ::mosek::fusion::Variable > _2003,monty::rc_ptr< ::mosek::fusion::Variable > _2004,int _2005);
static  monty::rc_ptr< ::mosek::fusion::Variable > stack(monty::rc_ptr< ::mosek::fusion::Variable > _2006,monty::rc_ptr< ::mosek::fusion::Variable > _2007,int _2008);
static  monty::rc_ptr< ::mosek::fusion::Variable > stack(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Variable >,1 > > _2009,int _2010);
static  monty::rc_ptr< ::mosek::fusion::Variable > promote(monty::rc_ptr< ::mosek::fusion::Variable > _2013,int _2014);
static  monty::rc_ptr< ::mosek::fusion::Variable > dstack(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Variable >,1 > > _2019,int _2020);
}; // struct Var;

struct p_ConstraintCache
{
ConstraintCache * _pubthis;
static mosek::fusion::p_ConstraintCache* _get_impl(mosek::fusion::ConstraintCache * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_ConstraintCache * _get_impl(mosek::fusion::ConstraintCache::t _inst) { return _get_impl(_inst.get()); }
p_ConstraintCache(ConstraintCache * _pubthis);
virtual ~p_ConstraintCache() { /* std::cout << "~p_ConstraintCache" << std::endl;*/ };
std::shared_ptr< monty::ndarray< int,1 > > barmatidx{};std::shared_ptr< monty::ndarray< int,1 > > barsubj{};std::shared_ptr< monty::ndarray< int,1 > > barsubi{};long long nbarnz{};long long nunordered{};std::shared_ptr< monty::ndarray< int,1 > > buffer_subi{};std::shared_ptr< monty::ndarray< int,1 > > buffer_subj{};std::shared_ptr< monty::ndarray< double,1 > > buffer_cof{};std::shared_ptr< monty::ndarray< double,1 > > bfix{};std::shared_ptr< monty::ndarray< double,1 > > cof{};std::shared_ptr< monty::ndarray< int,1 > > subi{};std::shared_ptr< monty::ndarray< int,1 > > subj{};long long nnz{};int nrows{};virtual void destroy();
static ConstraintCache::t _new_ConstraintCache(monty::rc_ptr< ::mosek::fusion::ConstraintCache > _2143);
void _initialize(monty::rc_ptr< ::mosek::fusion::ConstraintCache > _2143);
static ConstraintCache::t _new_ConstraintCache(std::shared_ptr< monty::ndarray< long long,1 > > _2144,std::shared_ptr< monty::ndarray< double,1 > > _2145,std::shared_ptr< monty::ndarray< int,1 > > _2146,std::shared_ptr< monty::ndarray< double,1 > > _2147,std::shared_ptr< monty::ndarray< int,1 > > _2148,std::shared_ptr< monty::ndarray< int,1 > > _2149,std::shared_ptr< monty::ndarray< int,1 > > _2150);
void _initialize(std::shared_ptr< monty::ndarray< long long,1 > > _2144,std::shared_ptr< monty::ndarray< double,1 > > _2145,std::shared_ptr< monty::ndarray< int,1 > > _2146,std::shared_ptr< monty::ndarray< double,1 > > _2147,std::shared_ptr< monty::ndarray< int,1 > > _2148,std::shared_ptr< monty::ndarray< int,1 > > _2149,std::shared_ptr< monty::ndarray< int,1 > > _2150);
virtual void unchecked_add_fx(std::shared_ptr< monty::ndarray< double,1 > > _2153) ;
virtual long long order_barentries() ;
virtual void add_bar(std::shared_ptr< monty::ndarray< int,1 > > _2163,std::shared_ptr< monty::ndarray< int,1 > > _2164,std::shared_ptr< monty::ndarray< int,1 > > _2165) ;
virtual void unchecked_add_l(std::shared_ptr< monty::ndarray< long long,1 > > _2171,std::shared_ptr< monty::ndarray< int,1 > > _2172,std::shared_ptr< monty::ndarray< double,1 > > _2173,std::shared_ptr< monty::ndarray< double,1 > > _2174) ;
virtual void add(std::shared_ptr< monty::ndarray< long long,1 > > _2183,std::shared_ptr< monty::ndarray< int,1 > > _2184,std::shared_ptr< monty::ndarray< double,1 > > _2185,std::shared_ptr< monty::ndarray< double,1 > > _2186) ;
virtual long long flush(std::shared_ptr< monty::ndarray< int,1 > > _2187,std::shared_ptr< monty::ndarray< int,1 > > _2188,std::shared_ptr< monty::ndarray< double,1 > > _2189,std::shared_ptr< monty::ndarray< double,1 > > _2190) ;
virtual long long numUnsorted() ;
virtual monty::rc_ptr< ::mosek::fusion::ConstraintCache > __mosek_2fusion_2ConstraintCache__clone() ;
}; // struct ConstraintCache;

struct p_Constraint
{
Constraint * _pubthis;
static mosek::fusion::p_Constraint* _get_impl(mosek::fusion::Constraint * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_Constraint * _get_impl(mosek::fusion::Constraint::t _inst) { return _get_impl(_inst.get()); }
p_Constraint(Constraint * _pubthis);
virtual ~p_Constraint() { /* std::cout << "~p_Constraint" << std::endl;*/ };
std::shared_ptr< monty::ndarray< int,1 > > nativeidxs{};std::shared_ptr< monty::ndarray< int,1 > > shape{};monty::rc_ptr< ::mosek::fusion::Model > model{};virtual void destroy();
static Constraint::t _new_Constraint(monty::rc_ptr< ::mosek::fusion::Constraint > _2264,monty::rc_ptr< ::mosek::fusion::Model > _2265);
void _initialize(monty::rc_ptr< ::mosek::fusion::Constraint > _2264,monty::rc_ptr< ::mosek::fusion::Model > _2265);
static Constraint::t _new_Constraint(monty::rc_ptr< ::mosek::fusion::Model > _2266,std::shared_ptr< monty::ndarray< int,1 > > _2267,std::shared_ptr< monty::ndarray< int,1 > > _2268);
void _initialize(monty::rc_ptr< ::mosek::fusion::Model > _2266,std::shared_ptr< monty::ndarray< int,1 > > _2267,std::shared_ptr< monty::ndarray< int,1 > > _2268);
virtual /* override */ std::string toString() ;
virtual void toStringArray(std::shared_ptr< monty::ndarray< long long,1 > > _2269,long long _2270,std::shared_ptr< monty::ndarray< std::string,1 > > _2271) ;
virtual std::shared_ptr< monty::ndarray< double,1 > > dual() ;
virtual std::shared_ptr< monty::ndarray< double,1 > > level() ;
virtual void values(bool _2274,int _2275,std::shared_ptr< monty::ndarray< double,1 > > _2276) ;
virtual void update(std::shared_ptr< monty::ndarray< double,1 > > _2277) ;
virtual void update(monty::rc_ptr< ::mosek::fusion::Expression > _2278) ;
virtual void update(monty::rc_ptr< ::mosek::fusion::Expression > _2282,monty::rc_ptr< ::mosek::fusion::Variable > _2283,bool _2284) ;
virtual void update(monty::rc_ptr< ::mosek::fusion::Expression > _2303,monty::rc_ptr< ::mosek::fusion::Variable > _2304) ;
virtual monty::rc_ptr< ::mosek::fusion::Model > __mosek_2fusion_2Constraint__get_model() ;
virtual int get_nd() ;
virtual long long size() ;
static  monty::rc_ptr< ::mosek::fusion::Constraint > stack(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Constraint >,1 > > _2307,int _2308);
static  monty::rc_ptr< ::mosek::fusion::Constraint > stack(monty::rc_ptr< ::mosek::fusion::Constraint > _2309,monty::rc_ptr< ::mosek::fusion::Constraint > _2310,monty::rc_ptr< ::mosek::fusion::Constraint > _2311,int _2312);
static  monty::rc_ptr< ::mosek::fusion::Constraint > stack(monty::rc_ptr< ::mosek::fusion::Constraint > _2313,monty::rc_ptr< ::mosek::fusion::Constraint > _2314,int _2315);
static  monty::rc_ptr< ::mosek::fusion::Constraint > hstack(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Constraint >,1 > > _2316);
static  monty::rc_ptr< ::mosek::fusion::Constraint > vstack(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Constraint >,1 > > _2317);
static  monty::rc_ptr< ::mosek::fusion::Constraint > hstack(monty::rc_ptr< ::mosek::fusion::Constraint > _2318,monty::rc_ptr< ::mosek::fusion::Constraint > _2319,monty::rc_ptr< ::mosek::fusion::Constraint > _2320);
static  monty::rc_ptr< ::mosek::fusion::Constraint > vstack(monty::rc_ptr< ::mosek::fusion::Constraint > _2321,monty::rc_ptr< ::mosek::fusion::Constraint > _2322,monty::rc_ptr< ::mosek::fusion::Constraint > _2323);
static  monty::rc_ptr< ::mosek::fusion::Constraint > hstack(monty::rc_ptr< ::mosek::fusion::Constraint > _2324,monty::rc_ptr< ::mosek::fusion::Constraint > _2325);
static  monty::rc_ptr< ::mosek::fusion::Constraint > vstack(monty::rc_ptr< ::mosek::fusion::Constraint > _2326,monty::rc_ptr< ::mosek::fusion::Constraint > _2327);
static  monty::rc_ptr< ::mosek::fusion::Constraint > dstack(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Constraint >,1 > > _2328,int _2329);
virtual monty::rc_ptr< ::mosek::fusion::Constraint > __mosek_2fusion_2Constraint__index(std::shared_ptr< monty::ndarray< int,1 > > _2380) ;
virtual monty::rc_ptr< ::mosek::fusion::Constraint > __mosek_2fusion_2Constraint__index(int _2387) ;
virtual monty::rc_ptr< ::mosek::fusion::Constraint > __mosek_2fusion_2Constraint__slice(std::shared_ptr< monty::ndarray< int,1 > > _2388,std::shared_ptr< monty::ndarray< int,1 > > _2389) ;
virtual monty::rc_ptr< ::mosek::fusion::Constraint > __mosek_2fusion_2Constraint__slice(int _2408,int _2409) ;
virtual int getND() ;
virtual int getSize() ;
virtual monty::rc_ptr< ::mosek::fusion::Model > __mosek_2fusion_2Constraint__getModel() ;
virtual std::shared_ptr< monty::ndarray< int,1 > > getShape() ;
}; // struct Constraint;

struct p_SliceConstraint : public ::mosek::fusion::p_Constraint
{
SliceConstraint * _pubthis;
static mosek::fusion::p_SliceConstraint* _get_impl(mosek::fusion::SliceConstraint * _inst){ return static_cast< mosek::fusion::p_SliceConstraint* >(mosek::fusion::p_Constraint::_get_impl(_inst)); }
static mosek::fusion::p_SliceConstraint * _get_impl(mosek::fusion::SliceConstraint::t _inst) { return _get_impl(_inst.get()); }
p_SliceConstraint(SliceConstraint * _pubthis);
virtual ~p_SliceConstraint() { /* std::cout << "~p_SliceConstraint" << std::endl;*/ };
virtual void destroy();
static SliceConstraint::t _new_SliceConstraint(monty::rc_ptr< ::mosek::fusion::Model > _2213,std::shared_ptr< monty::ndarray< int,1 > > _2214,std::shared_ptr< monty::ndarray< int,1 > > _2215);
void _initialize(monty::rc_ptr< ::mosek::fusion::Model > _2213,std::shared_ptr< monty::ndarray< int,1 > > _2214,std::shared_ptr< monty::ndarray< int,1 > > _2215);
virtual /* override */ std::string toString() ;
}; // struct SliceConstraint;

struct p_ModelConstraint : public ::mosek::fusion::p_Constraint
{
ModelConstraint * _pubthis;
static mosek::fusion::p_ModelConstraint* _get_impl(mosek::fusion::ModelConstraint * _inst){ return static_cast< mosek::fusion::p_ModelConstraint* >(mosek::fusion::p_Constraint::_get_impl(_inst)); }
static mosek::fusion::p_ModelConstraint * _get_impl(mosek::fusion::ModelConstraint::t _inst) { return _get_impl(_inst.get()); }
p_ModelConstraint(ModelConstraint * _pubthis);
virtual ~p_ModelConstraint() { /* std::cout << "~p_ModelConstraint" << std::endl;*/ };
std::shared_ptr< monty::ndarray< int,1 > > shape{};std::shared_ptr< monty::ndarray< int,1 > > nativeidxs{};bool names_flushed{};std::string name{};virtual void destroy();
static ModelConstraint::t _new_ModelConstraint(monty::rc_ptr< ::mosek::fusion::ModelConstraint > _2253,monty::rc_ptr< ::mosek::fusion::Model > _2254);
void _initialize(monty::rc_ptr< ::mosek::fusion::ModelConstraint > _2253,monty::rc_ptr< ::mosek::fusion::Model > _2254);
static ModelConstraint::t _new_ModelConstraint(monty::rc_ptr< ::mosek::fusion::Model > _2255,const std::string &  _2256,std::shared_ptr< monty::ndarray< int,1 > > _2257,std::shared_ptr< monty::ndarray< int,1 > > _2258);
void _initialize(monty::rc_ptr< ::mosek::fusion::Model > _2255,const std::string &  _2256,std::shared_ptr< monty::ndarray< int,1 > > _2257,std::shared_ptr< monty::ndarray< int,1 > > _2258);
virtual /* override */ std::string toString() ;
virtual void flushNames() ;
virtual monty::rc_ptr< ::mosek::fusion::ModelConstraint > __mosek_2fusion_2ModelConstraint__clone(monty::rc_ptr< ::mosek::fusion::Model > _2263) { throw monty::AbstractClassError("Call to abstract method"); }
}; // struct ModelConstraint;

struct p_LinearPSDConstraint : public ::mosek::fusion::p_ModelConstraint
{
LinearPSDConstraint * _pubthis;
static mosek::fusion::p_LinearPSDConstraint* _get_impl(mosek::fusion::LinearPSDConstraint * _inst){ return static_cast< mosek::fusion::p_LinearPSDConstraint* >(mosek::fusion::p_ModelConstraint::_get_impl(_inst)); }
static mosek::fusion::p_LinearPSDConstraint * _get_impl(mosek::fusion::LinearPSDConstraint::t _inst) { return _get_impl(_inst.get()); }
p_LinearPSDConstraint(LinearPSDConstraint * _pubthis);
virtual ~p_LinearPSDConstraint() { /* std::cout << "~p_LinearPSDConstraint" << std::endl;*/ };
int conedim{};std::shared_ptr< monty::ndarray< int,1 > > shape{};int conid{};std::shared_ptr< monty::ndarray< long long,1 > > slackidxs{};std::shared_ptr< monty::ndarray< int,1 > > nativeidxs{};virtual void destroy();
static LinearPSDConstraint::t _new_LinearPSDConstraint(monty::rc_ptr< ::mosek::fusion::LinearPSDConstraint > _2089,monty::rc_ptr< ::mosek::fusion::Model > _2090);
void _initialize(monty::rc_ptr< ::mosek::fusion::LinearPSDConstraint > _2089,monty::rc_ptr< ::mosek::fusion::Model > _2090);
static LinearPSDConstraint::t _new_LinearPSDConstraint(monty::rc_ptr< ::mosek::fusion::Model > _2091,const std::string &  _2092,int _2093,std::shared_ptr< monty::ndarray< int,1 > > _2094,int _2095,std::shared_ptr< monty::ndarray< int,1 > > _2096,std::shared_ptr< monty::ndarray< long long,1 > > _2097);
void _initialize(monty::rc_ptr< ::mosek::fusion::Model > _2091,const std::string &  _2092,int _2093,std::shared_ptr< monty::ndarray< int,1 > > _2094,int _2095,std::shared_ptr< monty::ndarray< int,1 > > _2096,std::shared_ptr< monty::ndarray< long long,1 > > _2097);
virtual void domainToString(long long _2098,monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > _2099) ;
virtual /* override */ monty::rc_ptr< ::mosek::fusion::ModelConstraint > __mosek_2fusion_2LinearPSDConstraint__clone(monty::rc_ptr< ::mosek::fusion::Model > _2103) ;
virtual monty::rc_ptr< ::mosek::fusion::ModelConstraint > __mosek_2fusion_2ModelConstraint__clone(monty::rc_ptr< ::mosek::fusion::Model > _2103) { return __mosek_2fusion_2LinearPSDConstraint__clone(_2103); }
}; // struct LinearPSDConstraint;

struct p_PSDConstraint : public ::mosek::fusion::p_ModelConstraint
{
PSDConstraint * _pubthis;
static mosek::fusion::p_PSDConstraint* _get_impl(mosek::fusion::PSDConstraint * _inst){ return static_cast< mosek::fusion::p_PSDConstraint* >(mosek::fusion::p_ModelConstraint::_get_impl(_inst)); }
static mosek::fusion::p_PSDConstraint * _get_impl(mosek::fusion::PSDConstraint::t _inst) { return _get_impl(_inst.get()); }
p_PSDConstraint(PSDConstraint * _pubthis);
virtual ~p_PSDConstraint() { /* std::cout << "~p_PSDConstraint" << std::endl;*/ };
bool names_flushed{};int conedim1{};int conedim0{};std::shared_ptr< monty::ndarray< int,1 > > shape{};std::string name{};std::shared_ptr< monty::ndarray< long long,1 > > slackidxs{};std::shared_ptr< monty::ndarray< int,1 > > nativeidxs{};int conid{};virtual void destroy();
static PSDConstraint::t _new_PSDConstraint(monty::rc_ptr< ::mosek::fusion::PSDConstraint > _2104,monty::rc_ptr< ::mosek::fusion::Model > _2105);
void _initialize(monty::rc_ptr< ::mosek::fusion::PSDConstraint > _2104,monty::rc_ptr< ::mosek::fusion::Model > _2105);
static PSDConstraint::t _new_PSDConstraint(monty::rc_ptr< ::mosek::fusion::Model > _2106,const std::string &  _2107,int _2108,std::shared_ptr< monty::ndarray< int,1 > > _2109,int _2110,int _2111,std::shared_ptr< monty::ndarray< long long,1 > > _2112,std::shared_ptr< monty::ndarray< int,1 > > _2113);
void _initialize(monty::rc_ptr< ::mosek::fusion::Model > _2106,const std::string &  _2107,int _2108,std::shared_ptr< monty::ndarray< int,1 > > _2109,int _2110,int _2111,std::shared_ptr< monty::ndarray< long long,1 > > _2112,std::shared_ptr< monty::ndarray< int,1 > > _2113);
virtual /* override */ std::string toString() ;
virtual /* override */ monty::rc_ptr< ::mosek::fusion::ModelConstraint > __mosek_2fusion_2PSDConstraint__clone(monty::rc_ptr< ::mosek::fusion::Model > _2114) ;
virtual monty::rc_ptr< ::mosek::fusion::ModelConstraint > __mosek_2fusion_2ModelConstraint__clone(monty::rc_ptr< ::mosek::fusion::Model > _2114) { return __mosek_2fusion_2PSDConstraint__clone(_2114); }
static  std::shared_ptr< monty::ndarray< int,1 > > computenidxs(std::shared_ptr< monty::ndarray< int,1 > > _2115,int _2116,int _2117,std::shared_ptr< monty::ndarray< int,1 > > _2118);
}; // struct PSDConstraint;

struct p_RangedConstraint : public ::mosek::fusion::p_ModelConstraint
{
RangedConstraint * _pubthis;
static mosek::fusion::p_RangedConstraint* _get_impl(mosek::fusion::RangedConstraint * _inst){ return static_cast< mosek::fusion::p_RangedConstraint* >(mosek::fusion::p_ModelConstraint::_get_impl(_inst)); }
static mosek::fusion::p_RangedConstraint * _get_impl(mosek::fusion::RangedConstraint::t _inst) { return _get_impl(_inst.get()); }
p_RangedConstraint(RangedConstraint * _pubthis);
virtual ~p_RangedConstraint() { /* std::cout << "~p_RangedConstraint" << std::endl;*/ };
std::shared_ptr< monty::ndarray< int,1 > > nativeidxs{};std::shared_ptr< monty::ndarray< int,1 > > shape{};virtual void destroy();
static RangedConstraint::t _new_RangedConstraint(monty::rc_ptr< ::mosek::fusion::RangedConstraint > _2217,monty::rc_ptr< ::mosek::fusion::Model > _2218);
void _initialize(monty::rc_ptr< ::mosek::fusion::RangedConstraint > _2217,monty::rc_ptr< ::mosek::fusion::Model > _2218);
static RangedConstraint::t _new_RangedConstraint(monty::rc_ptr< ::mosek::fusion::Model > _2219,const std::string &  _2220,std::shared_ptr< monty::ndarray< int,1 > > _2221,std::shared_ptr< monty::ndarray< int,1 > > _2222);
void _initialize(monty::rc_ptr< ::mosek::fusion::Model > _2219,const std::string &  _2220,std::shared_ptr< monty::ndarray< int,1 > > _2221,std::shared_ptr< monty::ndarray< int,1 > > _2222);
virtual /* override */ monty::rc_ptr< ::mosek::fusion::ModelConstraint > __mosek_2fusion_2RangedConstraint__clone(monty::rc_ptr< ::mosek::fusion::Model > _2223) ;
virtual monty::rc_ptr< ::mosek::fusion::ModelConstraint > __mosek_2fusion_2ModelConstraint__clone(monty::rc_ptr< ::mosek::fusion::Model > _2223) { return __mosek_2fusion_2RangedConstraint__clone(_2223); }
}; // struct RangedConstraint;

struct p_ConicConstraint : public ::mosek::fusion::p_ModelConstraint
{
ConicConstraint * _pubthis;
static mosek::fusion::p_ConicConstraint* _get_impl(mosek::fusion::ConicConstraint * _inst){ return static_cast< mosek::fusion::p_ConicConstraint* >(mosek::fusion::p_ModelConstraint::_get_impl(_inst)); }
static mosek::fusion::p_ConicConstraint * _get_impl(mosek::fusion::ConicConstraint::t _inst) { return _get_impl(_inst.get()); }
p_ConicConstraint(ConicConstraint * _pubthis);
virtual ~p_ConicConstraint() { /* std::cout << "~p_ConicConstraint" << std::endl;*/ };
std::shared_ptr< monty::ndarray< int,1 > > nativeslack{};std::shared_ptr< monty::ndarray< int,1 > > nativeidxs{};bool names_flushed{};std::string name{};std::shared_ptr< monty::ndarray< int,1 > > shape{};monty::rc_ptr< ::mosek::fusion::ConeDomain > dom{};int conid{};virtual void destroy();
static ConicConstraint::t _new_ConicConstraint(monty::rc_ptr< ::mosek::fusion::ConicConstraint > _2224,monty::rc_ptr< ::mosek::fusion::Model > _2225);
void _initialize(monty::rc_ptr< ::mosek::fusion::ConicConstraint > _2224,monty::rc_ptr< ::mosek::fusion::Model > _2225);
static ConicConstraint::t _new_ConicConstraint(monty::rc_ptr< ::mosek::fusion::Model > _2226,const std::string &  _2227,monty::rc_ptr< ::mosek::fusion::ConeDomain > _2228,std::shared_ptr< monty::ndarray< int,1 > > _2229,int _2230,std::shared_ptr< monty::ndarray< int,1 > > _2231,std::shared_ptr< monty::ndarray< int,1 > > _2232);
void _initialize(monty::rc_ptr< ::mosek::fusion::Model > _2226,const std::string &  _2227,monty::rc_ptr< ::mosek::fusion::ConeDomain > _2228,std::shared_ptr< monty::ndarray< int,1 > > _2229,int _2230,std::shared_ptr< monty::ndarray< int,1 > > _2231,std::shared_ptr< monty::ndarray< int,1 > > _2232);
virtual /* override */ void flushNames() ;
virtual /* override */ std::string toString() ;
virtual void domainToString(long long _2239,monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > _2240) ;
virtual /* override */ monty::rc_ptr< ::mosek::fusion::ModelConstraint > __mosek_2fusion_2ConicConstraint__clone(monty::rc_ptr< ::mosek::fusion::Model > _2241) ;
virtual monty::rc_ptr< ::mosek::fusion::ModelConstraint > __mosek_2fusion_2ModelConstraint__clone(monty::rc_ptr< ::mosek::fusion::Model > _2241) { return __mosek_2fusion_2ConicConstraint__clone(_2241); }
}; // struct ConicConstraint;

struct p_LinearConstraint : public ::mosek::fusion::p_ModelConstraint
{
LinearConstraint * _pubthis;
static mosek::fusion::p_LinearConstraint* _get_impl(mosek::fusion::LinearConstraint * _inst){ return static_cast< mosek::fusion::p_LinearConstraint* >(mosek::fusion::p_ModelConstraint::_get_impl(_inst)); }
static mosek::fusion::p_LinearConstraint * _get_impl(mosek::fusion::LinearConstraint::t _inst) { return _get_impl(_inst.get()); }
p_LinearConstraint(LinearConstraint * _pubthis);
virtual ~p_LinearConstraint() { /* std::cout << "~p_LinearConstraint" << std::endl;*/ };
std::string name{};int conid{};virtual void destroy();
static LinearConstraint::t _new_LinearConstraint(monty::rc_ptr< ::mosek::fusion::LinearConstraint > _2242,monty::rc_ptr< ::mosek::fusion::Model > _2243);
void _initialize(monty::rc_ptr< ::mosek::fusion::LinearConstraint > _2242,monty::rc_ptr< ::mosek::fusion::Model > _2243);
static LinearConstraint::t _new_LinearConstraint(monty::rc_ptr< ::mosek::fusion::Model > _2244,const std::string &  _2245,int _2246,std::shared_ptr< monty::ndarray< int,1 > > _2247,std::shared_ptr< monty::ndarray< int,1 > > _2248);
void _initialize(monty::rc_ptr< ::mosek::fusion::Model > _2244,const std::string &  _2245,int _2246,std::shared_ptr< monty::ndarray< int,1 > > _2247,std::shared_ptr< monty::ndarray< int,1 > > _2248);
virtual /* override */ std::string toString() ;
virtual void domainToString(long long _2250,monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > _2251) ;
virtual /* override */ monty::rc_ptr< ::mosek::fusion::ModelConstraint > __mosek_2fusion_2LinearConstraint__clone(monty::rc_ptr< ::mosek::fusion::Model > _2252) ;
virtual monty::rc_ptr< ::mosek::fusion::ModelConstraint > __mosek_2fusion_2ModelConstraint__clone(monty::rc_ptr< ::mosek::fusion::Model > _2252) { return __mosek_2fusion_2LinearConstraint__clone(_2252); }
}; // struct LinearConstraint;

struct p_Set
{
Set * _pubthis;
static mosek::fusion::p_Set* _get_impl(mosek::fusion::Set * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_Set * _get_impl(mosek::fusion::Set::t _inst) { return _get_impl(_inst.get()); }
p_Set(Set * _pubthis);
virtual ~p_Set() { /* std::cout << "~p_Set" << std::endl;*/ };
virtual void destroy();
static  long long size(std::shared_ptr< monty::ndarray< int,1 > > _2414);
static  bool match(std::shared_ptr< monty::ndarray< int,1 > > _2417,std::shared_ptr< monty::ndarray< int,1 > > _2418);
static  long long linearidx(std::shared_ptr< monty::ndarray< int,1 > > _2420,std::shared_ptr< monty::ndarray< int,1 > > _2421);
static  std::shared_ptr< monty::ndarray< int,1 > > idxtokey(std::shared_ptr< monty::ndarray< int,1 > > _2424,long long _2425);
static  void idxtokey(std::shared_ptr< monty::ndarray< int,1 > > _2427,long long _2428,std::shared_ptr< monty::ndarray< int,1 > > _2429);
static  std::string indexToString(std::shared_ptr< monty::ndarray< int,1 > > _2433,long long _2434);
static  std::string keyToString(std::shared_ptr< monty::ndarray< int,1 > > _2441);
static  void indexToKey(std::shared_ptr< monty::ndarray< int,1 > > _2444,long long _2445,std::shared_ptr< monty::ndarray< int,1 > > _2446);
static  std::shared_ptr< monty::ndarray< long long,1 > > strides(std::shared_ptr< monty::ndarray< int,1 > > _2450);
static  std::shared_ptr< monty::ndarray< int,1 > > make(std::shared_ptr< monty::ndarray< int,1 > > _2454,std::shared_ptr< monty::ndarray< int,1 > > _2455);
static  std::shared_ptr< monty::ndarray< int,1 > > make(std::shared_ptr< monty::ndarray< int,1 > > _2459);
static  std::shared_ptr< monty::ndarray< int,1 > > make(int _2461,int _2462,int _2463);
static  std::shared_ptr< monty::ndarray< int,1 > > make(int _2464,int _2465);
static  std::shared_ptr< monty::ndarray< int,1 > > make(int _2466);
static  std::shared_ptr< monty::ndarray< int,1 > > scalar();
static  std::shared_ptr< monty::ndarray< int,1 > > make(std::shared_ptr< monty::ndarray< std::string,1 > > _2467);
}; // struct Set;

struct p_ConeDomain
{
ConeDomain * _pubthis;
static mosek::fusion::p_ConeDomain* _get_impl(mosek::fusion::ConeDomain * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_ConeDomain * _get_impl(mosek::fusion::ConeDomain::t _inst) { return _get_impl(_inst.get()); }
p_ConeDomain(ConeDomain * _pubthis);
virtual ~p_ConeDomain() { /* std::cout << "~p_ConeDomain" << std::endl;*/ };
double alpha{};std::shared_ptr< monty::ndarray< int,1 > > shape{};bool int_flag{};bool axisset{};int axisidx{};mosek::fusion::QConeKey key{};virtual void destroy();
static ConeDomain::t _new_ConeDomain(mosek::fusion::QConeKey _2468,double _2469,std::shared_ptr< monty::ndarray< int,1 > > _2470);
void _initialize(mosek::fusion::QConeKey _2468,double _2469,std::shared_ptr< monty::ndarray< int,1 > > _2470);
static ConeDomain::t _new_ConeDomain(mosek::fusion::QConeKey _2471,std::shared_ptr< monty::ndarray< int,1 > > _2472);
void _initialize(mosek::fusion::QConeKey _2471,std::shared_ptr< monty::ndarray< int,1 > > _2472);
virtual bool match_shape(std::shared_ptr< monty::ndarray< int,1 > > _2473) ;
virtual monty::rc_ptr< ::mosek::fusion::ConeDomain > __mosek_2fusion_2ConeDomain__integral() ;
virtual bool axisIsSet() ;
virtual int getAxis() ;
virtual monty::rc_ptr< ::mosek::fusion::ConeDomain > __mosek_2fusion_2ConeDomain__axis(int _2474) ;
}; // struct ConeDomain;

struct p_LinPSDDomain
{
LinPSDDomain * _pubthis;
static mosek::fusion::p_LinPSDDomain* _get_impl(mosek::fusion::LinPSDDomain * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_LinPSDDomain * _get_impl(mosek::fusion::LinPSDDomain::t _inst) { return _get_impl(_inst.get()); }
p_LinPSDDomain(LinPSDDomain * _pubthis);
virtual ~p_LinPSDDomain() { /* std::cout << "~p_LinPSDDomain" << std::endl;*/ };
int conedim{};std::shared_ptr< monty::ndarray< int,1 > > shape{};virtual void destroy();
static LinPSDDomain::t _new_LinPSDDomain(std::shared_ptr< monty::ndarray< int,1 > > _2475,int _2476);
void _initialize(std::shared_ptr< monty::ndarray< int,1 > > _2475,int _2476);
static LinPSDDomain::t _new_LinPSDDomain(std::shared_ptr< monty::ndarray< int,1 > > _2477);
void _initialize(std::shared_ptr< monty::ndarray< int,1 > > _2477);
static LinPSDDomain::t _new_LinPSDDomain();
void _initialize();
}; // struct LinPSDDomain;

struct p_PSDDomain
{
PSDDomain * _pubthis;
static mosek::fusion::p_PSDDomain* _get_impl(mosek::fusion::PSDDomain * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_PSDDomain * _get_impl(mosek::fusion::PSDDomain::t _inst) { return _get_impl(_inst.get()); }
p_PSDDomain(PSDDomain * _pubthis);
virtual ~p_PSDDomain() { /* std::cout << "~p_PSDDomain" << std::endl;*/ };
bool axisIsSet{};int conedim2{};int conedim1{};mosek::fusion::PSDKey key{};std::shared_ptr< monty::ndarray< int,1 > > shape{};virtual void destroy();
static PSDDomain::t _new_PSDDomain(mosek::fusion::PSDKey _2478,std::shared_ptr< monty::ndarray< int,1 > > _2479,int _2480,int _2481);
void _initialize(mosek::fusion::PSDKey _2478,std::shared_ptr< monty::ndarray< int,1 > > _2479,int _2480,int _2481);
static PSDDomain::t _new_PSDDomain(mosek::fusion::PSDKey _2483,std::shared_ptr< monty::ndarray< int,1 > > _2484);
void _initialize(mosek::fusion::PSDKey _2483,std::shared_ptr< monty::ndarray< int,1 > > _2484);
static PSDDomain::t _new_PSDDomain(mosek::fusion::PSDKey _2485);
void _initialize(mosek::fusion::PSDKey _2485);
virtual monty::rc_ptr< ::mosek::fusion::PSDDomain > __mosek_2fusion_2PSDDomain__axis(int _2486,int _2487) ;
}; // struct PSDDomain;

struct p_RangeDomain
{
RangeDomain * _pubthis;
static mosek::fusion::p_RangeDomain* _get_impl(mosek::fusion::RangeDomain * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_RangeDomain * _get_impl(mosek::fusion::RangeDomain::t _inst) { return _get_impl(_inst.get()); }
p_RangeDomain(RangeDomain * _pubthis);
virtual ~p_RangeDomain() { /* std::cout << "~p_RangeDomain" << std::endl;*/ };
bool cardinal_flag{};bool scalable{};std::shared_ptr< monty::ndarray< double,1 > > ub{};std::shared_ptr< monty::ndarray< double,1 > > lb{};std::shared_ptr< monty::ndarray< int,2 > > sparsity{};bool empty{};std::shared_ptr< monty::ndarray< int,1 > > shape{};virtual void destroy();
static RangeDomain::t _new_RangeDomain(bool _2489,std::shared_ptr< monty::ndarray< double,1 > > _2490,std::shared_ptr< monty::ndarray< double,1 > > _2491,std::shared_ptr< monty::ndarray< int,1 > > _2492);
void _initialize(bool _2489,std::shared_ptr< monty::ndarray< double,1 > > _2490,std::shared_ptr< monty::ndarray< double,1 > > _2491,std::shared_ptr< monty::ndarray< int,1 > > _2492);
static RangeDomain::t _new_RangeDomain(bool _2493,std::shared_ptr< monty::ndarray< double,1 > > _2494,std::shared_ptr< monty::ndarray< double,1 > > _2495,std::shared_ptr< monty::ndarray< int,1 > > _2496,std::shared_ptr< monty::ndarray< int,2 > > _2497);
void _initialize(bool _2493,std::shared_ptr< monty::ndarray< double,1 > > _2494,std::shared_ptr< monty::ndarray< double,1 > > _2495,std::shared_ptr< monty::ndarray< int,1 > > _2496,std::shared_ptr< monty::ndarray< int,2 > > _2497);
static RangeDomain::t _new_RangeDomain(bool _2498,std::shared_ptr< monty::ndarray< double,1 > > _2499,std::shared_ptr< monty::ndarray< double,1 > > _2500,std::shared_ptr< monty::ndarray< int,1 > > _2501,std::shared_ptr< monty::ndarray< int,2 > > _2502,int _2503);
void _initialize(bool _2498,std::shared_ptr< monty::ndarray< double,1 > > _2499,std::shared_ptr< monty::ndarray< double,1 > > _2500,std::shared_ptr< monty::ndarray< int,1 > > _2501,std::shared_ptr< monty::ndarray< int,2 > > _2502,int _2503);
static RangeDomain::t _new_RangeDomain(monty::rc_ptr< ::mosek::fusion::RangeDomain > _2504);
void _initialize(monty::rc_ptr< ::mosek::fusion::RangeDomain > _2504);
virtual monty::rc_ptr< ::mosek::fusion::SymmetricRangeDomain > __mosek_2fusion_2RangeDomain__symmetric() ;
virtual monty::rc_ptr< ::mosek::fusion::RangeDomain > __mosek_2fusion_2RangeDomain__sparse(std::shared_ptr< monty::ndarray< int,2 > > _2505) ;
virtual monty::rc_ptr< ::mosek::fusion::RangeDomain > __mosek_2fusion_2RangeDomain__sparse(std::shared_ptr< monty::ndarray< int,1 > > _2508) ;
virtual monty::rc_ptr< ::mosek::fusion::RangeDomain > __mosek_2fusion_2RangeDomain__sparse() ;
virtual monty::rc_ptr< ::mosek::fusion::RangeDomain > __mosek_2fusion_2RangeDomain__integral() ;
virtual monty::rc_ptr< ::mosek::fusion::RangeDomain > __mosek_2fusion_2RangeDomain__withShape(int _2510,int _2511,int _2512) ;
virtual monty::rc_ptr< ::mosek::fusion::RangeDomain > __mosek_2fusion_2RangeDomain__withShape(int _2513,int _2514) ;
virtual monty::rc_ptr< ::mosek::fusion::RangeDomain > __mosek_2fusion_2RangeDomain__withShape(int _2515) ;
virtual monty::rc_ptr< ::mosek::fusion::RangeDomain > __mosek_2fusion_2RangeDomain__withShape(std::shared_ptr< monty::ndarray< int,1 > > _2516) ;
virtual bool match_shape(std::shared_ptr< monty::ndarray< int,1 > > _2517) ;
}; // struct RangeDomain;

struct p_SymmetricRangeDomain : public ::mosek::fusion::p_RangeDomain
{
SymmetricRangeDomain * _pubthis;
static mosek::fusion::p_SymmetricRangeDomain* _get_impl(mosek::fusion::SymmetricRangeDomain * _inst){ return static_cast< mosek::fusion::p_SymmetricRangeDomain* >(mosek::fusion::p_RangeDomain::_get_impl(_inst)); }
static mosek::fusion::p_SymmetricRangeDomain * _get_impl(mosek::fusion::SymmetricRangeDomain::t _inst) { return _get_impl(_inst.get()); }
p_SymmetricRangeDomain(SymmetricRangeDomain * _pubthis);
virtual ~p_SymmetricRangeDomain() { /* std::cout << "~p_SymmetricRangeDomain" << std::endl;*/ };
int dim{};virtual void destroy();
static SymmetricRangeDomain::t _new_SymmetricRangeDomain(monty::rc_ptr< ::mosek::fusion::RangeDomain > _2488);
void _initialize(monty::rc_ptr< ::mosek::fusion::RangeDomain > _2488);
}; // struct SymmetricRangeDomain;

struct p_SymmetricLinearDomain
{
SymmetricLinearDomain * _pubthis;
static mosek::fusion::p_SymmetricLinearDomain* _get_impl(mosek::fusion::SymmetricLinearDomain * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_SymmetricLinearDomain * _get_impl(mosek::fusion::SymmetricLinearDomain::t _inst) { return _get_impl(_inst.get()); }
p_SymmetricLinearDomain(SymmetricLinearDomain * _pubthis);
virtual ~p_SymmetricLinearDomain() { /* std::cout << "~p_SymmetricLinearDomain" << std::endl;*/ };
std::shared_ptr< monty::ndarray< int,2 > > sparsity{};bool cardinal_flag{};mosek::fusion::RelationKey key{};std::shared_ptr< monty::ndarray< int,1 > > shape{};monty::rc_ptr< ::mosek::fusion::LinearDomain > dom{};int dim{};virtual void destroy();
static SymmetricLinearDomain::t _new_SymmetricLinearDomain(monty::rc_ptr< ::mosek::fusion::LinearDomain > _2519);
void _initialize(monty::rc_ptr< ::mosek::fusion::LinearDomain > _2519);
virtual monty::rc_ptr< ::mosek::fusion::SymmetricLinearDomain > __mosek_2fusion_2SymmetricLinearDomain__sparse(std::shared_ptr< monty::ndarray< int,2 > > _2520) ;
virtual monty::rc_ptr< ::mosek::fusion::SymmetricLinearDomain > __mosek_2fusion_2SymmetricLinearDomain__sparse(std::shared_ptr< monty::ndarray< int,1 > > _2523) ;
virtual monty::rc_ptr< ::mosek::fusion::SymmetricLinearDomain > __mosek_2fusion_2SymmetricLinearDomain__integral() ;
virtual bool match_shape(std::shared_ptr< monty::ndarray< int,1 > > _2525) ;
}; // struct SymmetricLinearDomain;

struct p_LinearDomain
{
LinearDomain * _pubthis;
static mosek::fusion::p_LinearDomain* _get_impl(mosek::fusion::LinearDomain * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_LinearDomain * _get_impl(mosek::fusion::LinearDomain::t _inst) { return _get_impl(_inst.get()); }
p_LinearDomain(LinearDomain * _pubthis);
virtual ~p_LinearDomain() { /* std::cout << "~p_LinearDomain" << std::endl;*/ };
bool empty{};bool scalable{};std::shared_ptr< monty::ndarray< int,2 > > sparsity{};bool cardinal_flag{};mosek::fusion::RelationKey key{};std::shared_ptr< monty::ndarray< double,1 > > bnd{};std::shared_ptr< monty::ndarray< int,1 > > shape{};virtual void destroy();
static LinearDomain::t _new_LinearDomain(mosek::fusion::RelationKey _2527,bool _2528,std::shared_ptr< monty::ndarray< double,1 > > _2529,std::shared_ptr< monty::ndarray< int,1 > > _2530);
void _initialize(mosek::fusion::RelationKey _2527,bool _2528,std::shared_ptr< monty::ndarray< double,1 > > _2529,std::shared_ptr< monty::ndarray< int,1 > > _2530);
static LinearDomain::t _new_LinearDomain(mosek::fusion::RelationKey _2531,bool _2532,std::shared_ptr< monty::ndarray< double,1 > > _2533,std::shared_ptr< monty::ndarray< int,1 > > _2534,std::shared_ptr< monty::ndarray< int,2 > > _2535,int _2536);
void _initialize(mosek::fusion::RelationKey _2531,bool _2532,std::shared_ptr< monty::ndarray< double,1 > > _2533,std::shared_ptr< monty::ndarray< int,1 > > _2534,std::shared_ptr< monty::ndarray< int,2 > > _2535,int _2536);
static LinearDomain::t _new_LinearDomain(monty::rc_ptr< ::mosek::fusion::LinearDomain > _2537);
void _initialize(monty::rc_ptr< ::mosek::fusion::LinearDomain > _2537);
virtual monty::rc_ptr< ::mosek::fusion::SymmetricLinearDomain > __mosek_2fusion_2LinearDomain__symmetric() ;
virtual monty::rc_ptr< ::mosek::fusion::LinearDomain > __mosek_2fusion_2LinearDomain__sparse(std::shared_ptr< monty::ndarray< int,2 > > _2538) ;
virtual monty::rc_ptr< ::mosek::fusion::LinearDomain > __mosek_2fusion_2LinearDomain__sparse(std::shared_ptr< monty::ndarray< int,1 > > _2541) ;
virtual monty::rc_ptr< ::mosek::fusion::LinearDomain > __mosek_2fusion_2LinearDomain__sparse() ;
virtual monty::rc_ptr< ::mosek::fusion::LinearDomain > __mosek_2fusion_2LinearDomain__integral() ;
virtual monty::rc_ptr< ::mosek::fusion::LinearDomain > __mosek_2fusion_2LinearDomain__withShape(int _2543,int _2544,int _2545) ;
virtual monty::rc_ptr< ::mosek::fusion::LinearDomain > __mosek_2fusion_2LinearDomain__withShape(int _2546,int _2547) ;
virtual monty::rc_ptr< ::mosek::fusion::LinearDomain > __mosek_2fusion_2LinearDomain__withShape(int _2548) ;
virtual monty::rc_ptr< ::mosek::fusion::LinearDomain > __mosek_2fusion_2LinearDomain__withShape(std::shared_ptr< monty::ndarray< int,1 > > _2549) ;
virtual bool match_shape(std::shared_ptr< monty::ndarray< int,1 > > _2550) ;
}; // struct LinearDomain;

struct p_Domain
{
Domain * _pubthis;
static mosek::fusion::p_Domain* _get_impl(mosek::fusion::Domain * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_Domain * _get_impl(mosek::fusion::Domain::t _inst) { return _get_impl(_inst.get()); }
p_Domain(Domain * _pubthis);
virtual ~p_Domain() { /* std::cout << "~p_Domain" << std::endl;*/ };
virtual void destroy();
static  long long dimsize(std::shared_ptr< monty::ndarray< int,1 > > _2552);
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > mkRangedDomain(monty::rc_ptr< ::mosek::fusion::Matrix > _2555,monty::rc_ptr< ::mosek::fusion::Matrix > _2556);
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > mkRangedDomain(std::shared_ptr< monty::ndarray< double,2 > > _2585,std::shared_ptr< monty::ndarray< double,2 > > _2586);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > mkLinearDomain(mosek::fusion::RelationKey _2595,monty::rc_ptr< ::mosek::fusion::Matrix > _2596);
static  long long prod(std::shared_ptr< monty::ndarray< int,1 > > _2602);
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > inRange(bool _2605,std::shared_ptr< monty::ndarray< double,1 > > _2606,std::shared_ptr< monty::ndarray< double,1 > > _2607,std::shared_ptr< monty::ndarray< int,2 > > _2608,std::shared_ptr< monty::ndarray< int,1 > > _2609);
static  monty::rc_ptr< ::mosek::fusion::SymmetricRangeDomain > symmetric(monty::rc_ptr< ::mosek::fusion::RangeDomain > _2611);
static  monty::rc_ptr< ::mosek::fusion::SymmetricLinearDomain > symmetric(monty::rc_ptr< ::mosek::fusion::LinearDomain > _2612);
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > sparse(monty::rc_ptr< ::mosek::fusion::RangeDomain > _2613,std::shared_ptr< monty::ndarray< int,2 > > _2614);
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > sparse(monty::rc_ptr< ::mosek::fusion::RangeDomain > _2615,std::shared_ptr< monty::ndarray< int,1 > > _2616);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > sparse(monty::rc_ptr< ::mosek::fusion::LinearDomain > _2617,std::shared_ptr< monty::ndarray< int,2 > > _2618);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > sparse(monty::rc_ptr< ::mosek::fusion::LinearDomain > _2619,std::shared_ptr< monty::ndarray< int,1 > > _2620);
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > integral(monty::rc_ptr< ::mosek::fusion::RangeDomain > _2621);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > integral(monty::rc_ptr< ::mosek::fusion::LinearDomain > _2622);
static  monty::rc_ptr< ::mosek::fusion::ConeDomain > integral(monty::rc_ptr< ::mosek::fusion::ConeDomain > _2623);
static  monty::rc_ptr< ::mosek::fusion::ConeDomain > axis(monty::rc_ptr< ::mosek::fusion::ConeDomain > _2624,int _2625);
static  monty::rc_ptr< ::mosek::fusion::ConeDomain > inDPowerCone(double _2626,std::shared_ptr< monty::ndarray< int,1 > > _2627);
static  monty::rc_ptr< ::mosek::fusion::ConeDomain > inDPowerCone(double _2629,int _2630);
static  monty::rc_ptr< ::mosek::fusion::ConeDomain > inDPowerCone(double _2631);
static  monty::rc_ptr< ::mosek::fusion::ConeDomain > inPPowerCone(double _2632,std::shared_ptr< monty::ndarray< int,1 > > _2633);
static  monty::rc_ptr< ::mosek::fusion::ConeDomain > inPPowerCone(double _2635,int _2636);
static  monty::rc_ptr< ::mosek::fusion::ConeDomain > inPPowerCone(double _2637);
static  monty::rc_ptr< ::mosek::fusion::ConeDomain > inDExpCone(std::shared_ptr< monty::ndarray< int,1 > > _2638);
static  monty::rc_ptr< ::mosek::fusion::ConeDomain > inDExpCone(int _2640);
static  monty::rc_ptr< ::mosek::fusion::ConeDomain > inDExpCone();
static  monty::rc_ptr< ::mosek::fusion::ConeDomain > inPExpCone(std::shared_ptr< monty::ndarray< int,1 > > _2641);
static  monty::rc_ptr< ::mosek::fusion::ConeDomain > inPExpCone(int _2643);
static  monty::rc_ptr< ::mosek::fusion::ConeDomain > inPExpCone();
static  monty::rc_ptr< ::mosek::fusion::ConeDomain > inRotatedQCone(std::shared_ptr< monty::ndarray< int,1 > > _2644);
static  monty::rc_ptr< ::mosek::fusion::ConeDomain > inRotatedQCone(int _2646,int _2647);
static  monty::rc_ptr< ::mosek::fusion::ConeDomain > inRotatedQCone(int _2648);
static  monty::rc_ptr< ::mosek::fusion::ConeDomain > inRotatedQCone();
static  monty::rc_ptr< ::mosek::fusion::ConeDomain > inQCone(std::shared_ptr< monty::ndarray< int,1 > > _2649);
static  monty::rc_ptr< ::mosek::fusion::ConeDomain > inQCone(int _2651,int _2652);
static  monty::rc_ptr< ::mosek::fusion::ConeDomain > inQCone(int _2653);
static  monty::rc_ptr< ::mosek::fusion::ConeDomain > inQCone();
static  monty::rc_ptr< ::mosek::fusion::LinPSDDomain > isLinPSD(int _2654,int _2655);
static  monty::rc_ptr< ::mosek::fusion::LinPSDDomain > isLinPSD(int _2656);
static  monty::rc_ptr< ::mosek::fusion::LinPSDDomain > isLinPSD();
static  monty::rc_ptr< ::mosek::fusion::PSDDomain > isTrilPSD(int _2657,int _2658);
static  monty::rc_ptr< ::mosek::fusion::PSDDomain > isTrilPSD(int _2659);
static  monty::rc_ptr< ::mosek::fusion::PSDDomain > isTrilPSD();
static  monty::rc_ptr< ::mosek::fusion::PSDDomain > inPSDCone(int _2660,int _2661);
static  monty::rc_ptr< ::mosek::fusion::PSDDomain > inPSDCone(int _2662);
static  monty::rc_ptr< ::mosek::fusion::PSDDomain > inPSDCone();
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > binary();
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > binary(std::shared_ptr< monty::ndarray< int,1 > > _2663);
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > binary(int _2664,int _2665);
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > binary(int _2666);
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > inRange(monty::rc_ptr< ::mosek::fusion::Matrix > _2667,monty::rc_ptr< ::mosek::fusion::Matrix > _2668);
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > inRange(std::shared_ptr< monty::ndarray< double,2 > > _2669,std::shared_ptr< monty::ndarray< double,2 > > _2670);
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > inRange(std::shared_ptr< monty::ndarray< double,1 > > _2671,std::shared_ptr< monty::ndarray< double,1 > > _2672,std::shared_ptr< monty::ndarray< int,1 > > _2673);
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > inRange(std::shared_ptr< monty::ndarray< double,1 > > _2674,double _2675,std::shared_ptr< monty::ndarray< int,1 > > _2676);
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > inRange(double _2678,std::shared_ptr< monty::ndarray< double,1 > > _2679,std::shared_ptr< monty::ndarray< int,1 > > _2680);
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > inRange(double _2682,double _2683,std::shared_ptr< monty::ndarray< int,1 > > _2684);
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > inRange(std::shared_ptr< monty::ndarray< double,1 > > _2685,std::shared_ptr< monty::ndarray< double,1 > > _2686);
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > inRange(std::shared_ptr< monty::ndarray< double,1 > > _2687,double _2688);
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > inRange(double _2690,std::shared_ptr< monty::ndarray< double,1 > > _2691);
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > inRange(double _2693,double _2694);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > greaterThan(monty::rc_ptr< ::mosek::fusion::Matrix > _2695);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > greaterThan(std::shared_ptr< monty::ndarray< double,1 > > _2696,std::shared_ptr< monty::ndarray< int,1 > > _2697);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > greaterThan(std::shared_ptr< monty::ndarray< double,2 > > _2698);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > greaterThan(std::shared_ptr< monty::ndarray< double,1 > > _2701);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > greaterThan(double _2702,std::shared_ptr< monty::ndarray< int,1 > > _2703);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > greaterThan(double _2705,int _2706,int _2707);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > greaterThan(double _2709,int _2710);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > greaterThan(double _2712);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > lessThan(monty::rc_ptr< ::mosek::fusion::Matrix > _2713);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > lessThan(std::shared_ptr< monty::ndarray< double,1 > > _2714,std::shared_ptr< monty::ndarray< int,1 > > _2715);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > lessThan(std::shared_ptr< monty::ndarray< double,2 > > _2716);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > lessThan(std::shared_ptr< monty::ndarray< double,1 > > _2719);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > lessThan(double _2720,std::shared_ptr< monty::ndarray< int,1 > > _2721);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > lessThan(double _2722,int _2723,int _2724);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > lessThan(double _2725,int _2726);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > lessThan(double _2727);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > equalsTo(monty::rc_ptr< ::mosek::fusion::Matrix > _2728);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > equalsTo(std::shared_ptr< monty::ndarray< double,1 > > _2729,std::shared_ptr< monty::ndarray< int,1 > > _2730);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > equalsTo(std::shared_ptr< monty::ndarray< double,2 > > _2731);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > equalsTo(std::shared_ptr< monty::ndarray< double,1 > > _2734);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > equalsTo(double _2735,std::shared_ptr< monty::ndarray< int,1 > > _2736);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > equalsTo(double _2737,int _2738,int _2739);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > equalsTo(double _2740,int _2741);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > equalsTo(double _2742);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > unbounded(std::shared_ptr< monty::ndarray< int,1 > > _2743);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > unbounded(int _2745,int _2746);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > unbounded(int _2747);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > unbounded();
}; // struct Domain;

struct p_BaseExpression : public /*implements*/ virtual ::mosek::fusion::Expression
{
BaseExpression * _pubthis;
static mosek::fusion::p_BaseExpression* _get_impl(mosek::fusion::BaseExpression * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_BaseExpression * _get_impl(mosek::fusion::BaseExpression::t _inst) { return _get_impl(_inst.get()); }
p_BaseExpression(BaseExpression * _pubthis);
virtual ~p_BaseExpression() { /* std::cout << "~p_BaseExpression" << std::endl;*/ };
std::shared_ptr< monty::ndarray< int,1 > > shape{};virtual void destroy();
static BaseExpression::t _new_BaseExpression(std::shared_ptr< monty::ndarray< int,1 > > _4926);
void _initialize(std::shared_ptr< monty::ndarray< int,1 > > _4926);
virtual /* override */ std::string toString() ;
virtual monty::rc_ptr< ::mosek::fusion::FlatExpr > __mosek_2fusion_2BaseExpression__eval() ;
virtual monty::rc_ptr< ::mosek::fusion::FlatExpr > __mosek_2fusion_2Expression__eval() { return __mosek_2fusion_2BaseExpression__eval(); }
static  void storeexpr(monty::rc_ptr< ::mosek::fusion::WorkStack > _4942,std::shared_ptr< monty::ndarray< int,1 > > _4943,std::shared_ptr< monty::ndarray< int,1 > > _4944,std::shared_ptr< monty::ndarray< long long,1 > > _4945,std::shared_ptr< monty::ndarray< long long,1 > > _4946,std::shared_ptr< monty::ndarray< double,1 > > _4947,std::shared_ptr< monty::ndarray< double,1 > > _4948);
virtual void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _4959,monty::rc_ptr< ::mosek::fusion::WorkStack > _4960,monty::rc_ptr< ::mosek::fusion::WorkStack > _4961) { throw monty::AbstractClassError("Call to abstract method"); }
virtual monty::rc_ptr< ::mosek::fusion::Expression > __mosek_2fusion_2BaseExpression__pick(std::shared_ptr< monty::ndarray< int,2 > > _4962) ;
virtual monty::rc_ptr< ::mosek::fusion::Expression > __mosek_2fusion_2Expression__pick(std::shared_ptr< monty::ndarray< int,2 > > _4962) { return __mosek_2fusion_2BaseExpression__pick(_4962); }
virtual monty::rc_ptr< ::mosek::fusion::Expression > __mosek_2fusion_2BaseExpression__pick(std::shared_ptr< monty::ndarray< int,1 > > _4963) ;
virtual monty::rc_ptr< ::mosek::fusion::Expression > __mosek_2fusion_2Expression__pick(std::shared_ptr< monty::ndarray< int,1 > > _4963) { return __mosek_2fusion_2BaseExpression__pick(_4963); }
virtual monty::rc_ptr< ::mosek::fusion::Expression > __mosek_2fusion_2BaseExpression__index(std::shared_ptr< monty::ndarray< int,1 > > _4966) ;
virtual monty::rc_ptr< ::mosek::fusion::Expression > __mosek_2fusion_2Expression__index(std::shared_ptr< monty::ndarray< int,1 > > _4966) { return __mosek_2fusion_2BaseExpression__index(_4966); }
virtual monty::rc_ptr< ::mosek::fusion::Expression > __mosek_2fusion_2BaseExpression__index(int _4969) ;
virtual monty::rc_ptr< ::mosek::fusion::Expression > __mosek_2fusion_2Expression__index(int _4969) { return __mosek_2fusion_2BaseExpression__index(_4969); }
virtual monty::rc_ptr< ::mosek::fusion::Expression > __mosek_2fusion_2BaseExpression__slice(std::shared_ptr< monty::ndarray< int,1 > > _4971,std::shared_ptr< monty::ndarray< int,1 > > _4972) ;
virtual monty::rc_ptr< ::mosek::fusion::Expression > __mosek_2fusion_2Expression__slice(std::shared_ptr< monty::ndarray< int,1 > > _4971,std::shared_ptr< monty::ndarray< int,1 > > _4972) { return __mosek_2fusion_2BaseExpression__slice(_4971,_4972); }
virtual monty::rc_ptr< ::mosek::fusion::Expression > __mosek_2fusion_2BaseExpression__slice(int _4973,int _4974) ;
virtual monty::rc_ptr< ::mosek::fusion::Expression > __mosek_2fusion_2Expression__slice(int _4973,int _4974) { return __mosek_2fusion_2BaseExpression__slice(_4973,_4974); }
virtual long long getSize() ;
virtual int getND() ;
virtual int getDim(int _4975) ;
virtual std::shared_ptr< monty::ndarray< int,1 > > getShape() ;
}; // struct BaseExpression;

struct p_ExprConst : public ::mosek::fusion::p_BaseExpression
{
ExprConst * _pubthis;
static mosek::fusion::p_ExprConst* _get_impl(mosek::fusion::ExprConst * _inst){ return static_cast< mosek::fusion::p_ExprConst* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprConst * _get_impl(mosek::fusion::ExprConst::t _inst) { return _get_impl(_inst.get()); }
p_ExprConst(ExprConst * _pubthis);
virtual ~p_ExprConst() { /* std::cout << "~p_ExprConst" << std::endl;*/ };
std::shared_ptr< monty::ndarray< long long,1 > > sparsity{};std::shared_ptr< monty::ndarray< double,1 > > bfix{};virtual void destroy();
static ExprConst::t _new_ExprConst(std::shared_ptr< monty::ndarray< int,1 > > _2748,std::shared_ptr< monty::ndarray< long long,1 > > _2749,std::shared_ptr< monty::ndarray< double,1 > > _2750);
void _initialize(std::shared_ptr< monty::ndarray< int,1 > > _2748,std::shared_ptr< monty::ndarray< long long,1 > > _2749,std::shared_ptr< monty::ndarray< double,1 > > _2750);
static ExprConst::t _new_ExprConst(std::shared_ptr< monty::ndarray< int,1 > > _2751,std::shared_ptr< monty::ndarray< long long,1 > > _2752,double _2753);
void _initialize(std::shared_ptr< monty::ndarray< int,1 > > _2751,std::shared_ptr< monty::ndarray< long long,1 > > _2752,double _2753);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _2756,monty::rc_ptr< ::mosek::fusion::WorkStack > _2757,monty::rc_ptr< ::mosek::fusion::WorkStack > _2758) ;
static  void validate(std::shared_ptr< monty::ndarray< int,1 > > _2776,std::shared_ptr< monty::ndarray< double,1 > > _2777,std::shared_ptr< monty::ndarray< long long,1 > > _2778);
virtual /* override */ std::string toString() ;
}; // struct ExprConst;

struct p_ExprPick : public ::mosek::fusion::p_BaseExpression
{
ExprPick * _pubthis;
static mosek::fusion::p_ExprPick* _get_impl(mosek::fusion::ExprPick * _inst){ return static_cast< mosek::fusion::p_ExprPick* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprPick * _get_impl(mosek::fusion::ExprPick::t _inst) { return _get_impl(_inst.get()); }
p_ExprPick(ExprPick * _pubthis);
virtual ~p_ExprPick() { /* std::cout << "~p_ExprPick" << std::endl;*/ };
std::shared_ptr< monty::ndarray< long long,1 > > idxs{};monty::rc_ptr< ::mosek::fusion::Expression > expr{};virtual void destroy();
static ExprPick::t _new_ExprPick(monty::rc_ptr< ::mosek::fusion::Expression > _2782,std::shared_ptr< monty::ndarray< int,2 > > _2783);
void _initialize(monty::rc_ptr< ::mosek::fusion::Expression > _2782,std::shared_ptr< monty::ndarray< int,2 > > _2783);
static ExprPick::t _new_ExprPick(monty::rc_ptr< ::mosek::fusion::Expression > _2795,std::shared_ptr< monty::ndarray< long long,1 > > _2796);
void _initialize(monty::rc_ptr< ::mosek::fusion::Expression > _2795,std::shared_ptr< monty::ndarray< long long,1 > > _2796);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _2801,monty::rc_ptr< ::mosek::fusion::WorkStack > _2802,monty::rc_ptr< ::mosek::fusion::WorkStack > _2803) ;
virtual /* override */ std::string toString() ;
}; // struct ExprPick;

struct p_ExprSlice : public ::mosek::fusion::p_BaseExpression
{
ExprSlice * _pubthis;
static mosek::fusion::p_ExprSlice* _get_impl(mosek::fusion::ExprSlice * _inst){ return static_cast< mosek::fusion::p_ExprSlice* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprSlice * _get_impl(mosek::fusion::ExprSlice::t _inst) { return _get_impl(_inst.get()); }
p_ExprSlice(ExprSlice * _pubthis);
virtual ~p_ExprSlice() { /* std::cout << "~p_ExprSlice" << std::endl;*/ };
std::shared_ptr< monty::ndarray< int,1 > > last{};std::shared_ptr< monty::ndarray< int,1 > > first{};monty::rc_ptr< ::mosek::fusion::Expression > expr{};virtual void destroy();
static ExprSlice::t _new_ExprSlice(monty::rc_ptr< ::mosek::fusion::Expression > _2855,std::shared_ptr< monty::ndarray< int,1 > > _2856,std::shared_ptr< monty::ndarray< int,1 > > _2857);
void _initialize(monty::rc_ptr< ::mosek::fusion::Expression > _2855,std::shared_ptr< monty::ndarray< int,1 > > _2856,std::shared_ptr< monty::ndarray< int,1 > > _2857);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _2858,monty::rc_ptr< ::mosek::fusion::WorkStack > _2859,monty::rc_ptr< ::mosek::fusion::WorkStack > _2860) ;
static  std::shared_ptr< monty::ndarray< int,1 > > makeShape(std::shared_ptr< monty::ndarray< int,1 > > _2914,std::shared_ptr< monty::ndarray< int,1 > > _2915,std::shared_ptr< monty::ndarray< int,1 > > _2916);
virtual /* override */ std::string toString() ;
}; // struct ExprSlice;

struct p_ExprPermuteDims : public ::mosek::fusion::p_BaseExpression
{
ExprPermuteDims * _pubthis;
static mosek::fusion::p_ExprPermuteDims* _get_impl(mosek::fusion::ExprPermuteDims * _inst){ return static_cast< mosek::fusion::p_ExprPermuteDims* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprPermuteDims * _get_impl(mosek::fusion::ExprPermuteDims::t _inst) { return _get_impl(_inst.get()); }
p_ExprPermuteDims(ExprPermuteDims * _pubthis);
virtual ~p_ExprPermuteDims() { /* std::cout << "~p_ExprPermuteDims" << std::endl;*/ };
std::shared_ptr< monty::ndarray< int,1 > > dperm{};monty::rc_ptr< ::mosek::fusion::Expression > expr{};virtual void destroy();
static ExprPermuteDims::t _new_ExprPermuteDims(std::shared_ptr< monty::ndarray< int,1 > > _2921,monty::rc_ptr< ::mosek::fusion::Expression > _2922);
void _initialize(std::shared_ptr< monty::ndarray< int,1 > > _2921,monty::rc_ptr< ::mosek::fusion::Expression > _2922);
static ExprPermuteDims::t _new_ExprPermuteDims(std::shared_ptr< monty::ndarray< int,1 > > _2928,monty::rc_ptr< ::mosek::fusion::Expression > _2929,int _2930);
void _initialize(std::shared_ptr< monty::ndarray< int,1 > > _2928,monty::rc_ptr< ::mosek::fusion::Expression > _2929,int _2930);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _2931,monty::rc_ptr< ::mosek::fusion::WorkStack > _2932,monty::rc_ptr< ::mosek::fusion::WorkStack > _2933) ;
static  std::shared_ptr< monty::ndarray< int,1 > > computeshape(std::shared_ptr< monty::ndarray< int,1 > > _2985,std::shared_ptr< monty::ndarray< int,1 > > _2986);
}; // struct ExprPermuteDims;

struct p_ExprTranspose : public ::mosek::fusion::p_BaseExpression
{
ExprTranspose * _pubthis;
static mosek::fusion::p_ExprTranspose* _get_impl(mosek::fusion::ExprTranspose * _inst){ return static_cast< mosek::fusion::p_ExprTranspose* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprTranspose * _get_impl(mosek::fusion::ExprTranspose::t _inst) { return _get_impl(_inst.get()); }
p_ExprTranspose(ExprTranspose * _pubthis);
virtual ~p_ExprTranspose() { /* std::cout << "~p_ExprTranspose" << std::endl;*/ };
monty::rc_ptr< ::mosek::fusion::Expression > expr{};virtual void destroy();
static ExprTranspose::t _new_ExprTranspose(monty::rc_ptr< ::mosek::fusion::Expression > _2988);
void _initialize(monty::rc_ptr< ::mosek::fusion::Expression > _2988);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _2989,monty::rc_ptr< ::mosek::fusion::WorkStack > _2990,monty::rc_ptr< ::mosek::fusion::WorkStack > _2991) ;
virtual /* override */ std::string toString() ;
static  std::shared_ptr< monty::ndarray< int,1 > > transposeShape(std::shared_ptr< monty::ndarray< int,1 > > _3035);
}; // struct ExprTranspose;

struct p_ExprStack : public ::mosek::fusion::p_BaseExpression
{
ExprStack * _pubthis;
static mosek::fusion::p_ExprStack* _get_impl(mosek::fusion::ExprStack * _inst){ return static_cast< mosek::fusion::p_ExprStack* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprStack * _get_impl(mosek::fusion::ExprStack::t _inst) { return _get_impl(_inst.get()); }
p_ExprStack(ExprStack * _pubthis);
virtual ~p_ExprStack() { /* std::cout << "~p_ExprStack" << std::endl;*/ };
int dim{};std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Expression >,1 > > exprs{};virtual void destroy();
static ExprStack::t _new_ExprStack(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Expression >,1 > > _3036,int _3037);
void _initialize(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Expression >,1 > > _3036,int _3037);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _3039,monty::rc_ptr< ::mosek::fusion::WorkStack > _3040,monty::rc_ptr< ::mosek::fusion::WorkStack > _3041) ;
static  std::shared_ptr< monty::ndarray< int,1 > > getshape(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Expression >,1 > > _3189,int _3190);
virtual /* override */ std::string toString() ;
}; // struct ExprStack;

struct p_ExprInner : public ::mosek::fusion::p_BaseExpression
{
ExprInner * _pubthis;
static mosek::fusion::p_ExprInner* _get_impl(mosek::fusion::ExprInner * _inst){ return static_cast< mosek::fusion::p_ExprInner* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprInner * _get_impl(mosek::fusion::ExprInner::t _inst) { return _get_impl(_inst.get()); }
p_ExprInner(ExprInner * _pubthis);
virtual ~p_ExprInner() { /* std::cout << "~p_ExprInner" << std::endl;*/ };
std::shared_ptr< monty::ndarray< double,1 > > vcof{};std::shared_ptr< monty::ndarray< long long,1 > > vsub{};monty::rc_ptr< ::mosek::fusion::Expression > expr{};virtual void destroy();
static ExprInner::t _new_ExprInner(monty::rc_ptr< ::mosek::fusion::Expression > _3204,std::shared_ptr< monty::ndarray< long long,1 > > _3205,std::shared_ptr< monty::ndarray< double,1 > > _3206);
void _initialize(monty::rc_ptr< ::mosek::fusion::Expression > _3204,std::shared_ptr< monty::ndarray< long long,1 > > _3205,std::shared_ptr< monty::ndarray< double,1 > > _3206);
static ExprInner::t _new_ExprInner(monty::rc_ptr< ::mosek::fusion::Expression > _3212,std::shared_ptr< monty::ndarray< double,1 > > _3213);
void _initialize(monty::rc_ptr< ::mosek::fusion::Expression > _3212,std::shared_ptr< monty::ndarray< double,1 > > _3213);
static ExprInner::t _new_ExprInner(monty::rc_ptr< ::mosek::fusion::Expression > _3215,std::shared_ptr< monty::ndarray< int,2 > > _3216,std::shared_ptr< monty::ndarray< double,1 > > _3217);
void _initialize(monty::rc_ptr< ::mosek::fusion::Expression > _3215,std::shared_ptr< monty::ndarray< int,2 > > _3216,std::shared_ptr< monty::ndarray< double,1 > > _3217);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _3218,monty::rc_ptr< ::mosek::fusion::WorkStack > _3219,monty::rc_ptr< ::mosek::fusion::WorkStack > _3220) ;
static  std::shared_ptr< monty::ndarray< long long,1 > > range(int _3256);
static  std::shared_ptr< monty::ndarray< long long,1 > > convert(std::shared_ptr< monty::ndarray< int,1 > > _3258,std::shared_ptr< monty::ndarray< int,2 > > _3259);
virtual /* override */ std::string toString() ;
}; // struct ExprInner;

struct p_ExprMulDiagRight : public ::mosek::fusion::p_BaseExpression
{
ExprMulDiagRight * _pubthis;
static mosek::fusion::p_ExprMulDiagRight* _get_impl(mosek::fusion::ExprMulDiagRight * _inst){ return static_cast< mosek::fusion::p_ExprMulDiagRight* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprMulDiagRight * _get_impl(mosek::fusion::ExprMulDiagRight::t _inst) { return _get_impl(_inst.get()); }
p_ExprMulDiagRight(ExprMulDiagRight * _pubthis);
virtual ~p_ExprMulDiagRight() { /* std::cout << "~p_ExprMulDiagRight" << std::endl;*/ };
monty::rc_ptr< ::mosek::fusion::Expression > expr{};std::shared_ptr< monty::ndarray< double,1 > > mval{};std::shared_ptr< monty::ndarray< int,1 > > msubj{};std::shared_ptr< monty::ndarray< int,1 > > msubi{};int mdim1{};int mdim0{};virtual void destroy();
static ExprMulDiagRight::t _new_ExprMulDiagRight(int _3266,int _3267,std::shared_ptr< monty::ndarray< int,1 > > _3268,std::shared_ptr< monty::ndarray< int,1 > > _3269,std::shared_ptr< monty::ndarray< double,1 > > _3270,monty::rc_ptr< ::mosek::fusion::Expression > _3271,int _3272);
void _initialize(int _3266,int _3267,std::shared_ptr< monty::ndarray< int,1 > > _3268,std::shared_ptr< monty::ndarray< int,1 > > _3269,std::shared_ptr< monty::ndarray< double,1 > > _3270,monty::rc_ptr< ::mosek::fusion::Expression > _3271,int _3272);
static ExprMulDiagRight::t _new_ExprMulDiagRight(int _3273,int _3274,std::shared_ptr< monty::ndarray< int,1 > > _3275,std::shared_ptr< monty::ndarray< int,1 > > _3276,std::shared_ptr< monty::ndarray< double,1 > > _3277,monty::rc_ptr< ::mosek::fusion::Expression > _3278);
void _initialize(int _3273,int _3274,std::shared_ptr< monty::ndarray< int,1 > > _3275,std::shared_ptr< monty::ndarray< int,1 > > _3276,std::shared_ptr< monty::ndarray< double,1 > > _3277,monty::rc_ptr< ::mosek::fusion::Expression > _3278);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _3279,monty::rc_ptr< ::mosek::fusion::WorkStack > _3280,monty::rc_ptr< ::mosek::fusion::WorkStack > _3281) ;
static  int validate(int _3373,int _3374,std::shared_ptr< monty::ndarray< int,1 > > _3375,std::shared_ptr< monty::ndarray< int,1 > > _3376,std::shared_ptr< monty::ndarray< double,1 > > _3377,monty::rc_ptr< ::mosek::fusion::Expression > _3378);
virtual /* override */ std::string toString() ;
}; // struct ExprMulDiagRight;

struct p_ExprMulDiagLeft : public ::mosek::fusion::p_BaseExpression
{
ExprMulDiagLeft * _pubthis;
static mosek::fusion::p_ExprMulDiagLeft* _get_impl(mosek::fusion::ExprMulDiagLeft * _inst){ return static_cast< mosek::fusion::p_ExprMulDiagLeft* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprMulDiagLeft * _get_impl(mosek::fusion::ExprMulDiagLeft::t _inst) { return _get_impl(_inst.get()); }
p_ExprMulDiagLeft(ExprMulDiagLeft * _pubthis);
virtual ~p_ExprMulDiagLeft() { /* std::cout << "~p_ExprMulDiagLeft" << std::endl;*/ };
monty::rc_ptr< ::mosek::fusion::Expression > expr{};std::shared_ptr< monty::ndarray< double,1 > > mval{};std::shared_ptr< monty::ndarray< int,1 > > msubj{};std::shared_ptr< monty::ndarray< int,1 > > msubi{};int mdim1{};int mdim0{};virtual void destroy();
static ExprMulDiagLeft::t _new_ExprMulDiagLeft(int _3387,int _3388,std::shared_ptr< monty::ndarray< int,1 > > _3389,std::shared_ptr< monty::ndarray< int,1 > > _3390,std::shared_ptr< monty::ndarray< double,1 > > _3391,monty::rc_ptr< ::mosek::fusion::Expression > _3392,int _3393);
void _initialize(int _3387,int _3388,std::shared_ptr< monty::ndarray< int,1 > > _3389,std::shared_ptr< monty::ndarray< int,1 > > _3390,std::shared_ptr< monty::ndarray< double,1 > > _3391,monty::rc_ptr< ::mosek::fusion::Expression > _3392,int _3393);
static ExprMulDiagLeft::t _new_ExprMulDiagLeft(int _3394,int _3395,std::shared_ptr< monty::ndarray< int,1 > > _3396,std::shared_ptr< monty::ndarray< int,1 > > _3397,std::shared_ptr< monty::ndarray< double,1 > > _3398,monty::rc_ptr< ::mosek::fusion::Expression > _3399);
void _initialize(int _3394,int _3395,std::shared_ptr< monty::ndarray< int,1 > > _3396,std::shared_ptr< monty::ndarray< int,1 > > _3397,std::shared_ptr< monty::ndarray< double,1 > > _3398,monty::rc_ptr< ::mosek::fusion::Expression > _3399);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _3400,monty::rc_ptr< ::mosek::fusion::WorkStack > _3401,monty::rc_ptr< ::mosek::fusion::WorkStack > _3402) ;
static  int validate(int _3487,int _3488,std::shared_ptr< monty::ndarray< int,1 > > _3489,std::shared_ptr< monty::ndarray< int,1 > > _3490,std::shared_ptr< monty::ndarray< double,1 > > _3491,monty::rc_ptr< ::mosek::fusion::Expression > _3492);
virtual /* override */ std::string toString() ;
}; // struct ExprMulDiagLeft;

struct p_ExprMulElement : public ::mosek::fusion::p_BaseExpression
{
ExprMulElement * _pubthis;
static mosek::fusion::p_ExprMulElement* _get_impl(mosek::fusion::ExprMulElement * _inst){ return static_cast< mosek::fusion::p_ExprMulElement* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprMulElement * _get_impl(mosek::fusion::ExprMulElement::t _inst) { return _get_impl(_inst.get()); }
p_ExprMulElement(ExprMulElement * _pubthis);
virtual ~p_ExprMulElement() { /* std::cout << "~p_ExprMulElement" << std::endl;*/ };
monty::rc_ptr< ::mosek::fusion::Expression > expr{};std::shared_ptr< monty::ndarray< long long,1 > > msp{};std::shared_ptr< monty::ndarray< double,1 > > mcof{};virtual void destroy();
static ExprMulElement::t _new_ExprMulElement(std::shared_ptr< monty::ndarray< double,1 > > _3501,std::shared_ptr< monty::ndarray< long long,1 > > _3502,monty::rc_ptr< ::mosek::fusion::Expression > _3503);
void _initialize(std::shared_ptr< monty::ndarray< double,1 > > _3501,std::shared_ptr< monty::ndarray< long long,1 > > _3502,monty::rc_ptr< ::mosek::fusion::Expression > _3503);
static ExprMulElement::t _new_ExprMulElement(std::shared_ptr< monty::ndarray< double,1 > > _3510,std::shared_ptr< monty::ndarray< long long,1 > > _3511,monty::rc_ptr< ::mosek::fusion::Expression > _3512,int _3513);
void _initialize(std::shared_ptr< monty::ndarray< double,1 > > _3510,std::shared_ptr< monty::ndarray< long long,1 > > _3511,monty::rc_ptr< ::mosek::fusion::Expression > _3512,int _3513);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _3514,monty::rc_ptr< ::mosek::fusion::WorkStack > _3515,monty::rc_ptr< ::mosek::fusion::WorkStack > _3516) ;
virtual /* override */ std::string toString() ;
}; // struct ExprMulElement;

struct p_ExprMulScalarConst : public ::mosek::fusion::p_BaseExpression
{
ExprMulScalarConst * _pubthis;
static mosek::fusion::p_ExprMulScalarConst* _get_impl(mosek::fusion::ExprMulScalarConst * _inst){ return static_cast< mosek::fusion::p_ExprMulScalarConst* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprMulScalarConst * _get_impl(mosek::fusion::ExprMulScalarConst::t _inst) { return _get_impl(_inst.get()); }
p_ExprMulScalarConst(ExprMulScalarConst * _pubthis);
virtual ~p_ExprMulScalarConst() { /* std::cout << "~p_ExprMulScalarConst" << std::endl;*/ };
monty::rc_ptr< ::mosek::fusion::Expression > expr{};double c{};virtual void destroy();
static ExprMulScalarConst::t _new_ExprMulScalarConst(double _3563,monty::rc_ptr< ::mosek::fusion::Expression > _3564);
void _initialize(double _3563,monty::rc_ptr< ::mosek::fusion::Expression > _3564);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _3565,monty::rc_ptr< ::mosek::fusion::WorkStack > _3566,monty::rc_ptr< ::mosek::fusion::WorkStack > _3567) ;
virtual /* override */ std::string toString() ;
}; // struct ExprMulScalarConst;

struct p_ExprScalarMul : public ::mosek::fusion::p_BaseExpression
{
ExprScalarMul * _pubthis;
static mosek::fusion::p_ExprScalarMul* _get_impl(mosek::fusion::ExprScalarMul * _inst){ return static_cast< mosek::fusion::p_ExprScalarMul* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprScalarMul * _get_impl(mosek::fusion::ExprScalarMul::t _inst) { return _get_impl(_inst.get()); }
p_ExprScalarMul(ExprScalarMul * _pubthis);
virtual ~p_ExprScalarMul() { /* std::cout << "~p_ExprScalarMul" << std::endl;*/ };
monty::rc_ptr< ::mosek::fusion::Expression > expr{};std::shared_ptr< monty::ndarray< double,1 > > mval{};std::shared_ptr< monty::ndarray< int,1 > > msubj{};std::shared_ptr< monty::ndarray< int,1 > > msubi{};int mdim1{};int mdim0{};virtual void destroy();
static ExprScalarMul::t _new_ExprScalarMul(int _3575,int _3576,std::shared_ptr< monty::ndarray< int,1 > > _3577,std::shared_ptr< monty::ndarray< int,1 > > _3578,std::shared_ptr< monty::ndarray< double,1 > > _3579,monty::rc_ptr< ::mosek::fusion::Expression > _3580,int _3581);
void _initialize(int _3575,int _3576,std::shared_ptr< monty::ndarray< int,1 > > _3577,std::shared_ptr< monty::ndarray< int,1 > > _3578,std::shared_ptr< monty::ndarray< double,1 > > _3579,monty::rc_ptr< ::mosek::fusion::Expression > _3580,int _3581);
static ExprScalarMul::t _new_ExprScalarMul(int _3582,int _3583,std::shared_ptr< monty::ndarray< int,1 > > _3584,std::shared_ptr< monty::ndarray< int,1 > > _3585,std::shared_ptr< monty::ndarray< double,1 > > _3586,monty::rc_ptr< ::mosek::fusion::Expression > _3587);
void _initialize(int _3582,int _3583,std::shared_ptr< monty::ndarray< int,1 > > _3584,std::shared_ptr< monty::ndarray< int,1 > > _3585,std::shared_ptr< monty::ndarray< double,1 > > _3586,monty::rc_ptr< ::mosek::fusion::Expression > _3587);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _3588,monty::rc_ptr< ::mosek::fusion::WorkStack > _3589,monty::rc_ptr< ::mosek::fusion::WorkStack > _3590) ;
static  int validate(int _3615,int _3616,std::shared_ptr< monty::ndarray< int,1 > > _3617,std::shared_ptr< monty::ndarray< int,1 > > _3618,std::shared_ptr< monty::ndarray< double,1 > > _3619,monty::rc_ptr< ::mosek::fusion::Expression > _3620);
virtual /* override */ std::string toString() ;
}; // struct ExprScalarMul;

struct p_ExprMulRight : public ::mosek::fusion::p_BaseExpression
{
ExprMulRight * _pubthis;
static mosek::fusion::p_ExprMulRight* _get_impl(mosek::fusion::ExprMulRight * _inst){ return static_cast< mosek::fusion::p_ExprMulRight* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprMulRight * _get_impl(mosek::fusion::ExprMulRight::t _inst) { return _get_impl(_inst.get()); }
p_ExprMulRight(ExprMulRight * _pubthis);
virtual ~p_ExprMulRight() { /* std::cout << "~p_ExprMulRight" << std::endl;*/ };
monty::rc_ptr< ::mosek::fusion::Expression > expr{};std::shared_ptr< monty::ndarray< double,1 > > mval{};std::shared_ptr< monty::ndarray< int,1 > > msubj{};std::shared_ptr< monty::ndarray< int,1 > > msubi{};int mdim1{};int mdim0{};virtual void destroy();
static ExprMulRight::t _new_ExprMulRight(int _3627,int _3628,std::shared_ptr< monty::ndarray< int,1 > > _3629,std::shared_ptr< monty::ndarray< int,1 > > _3630,std::shared_ptr< monty::ndarray< double,1 > > _3631,monty::rc_ptr< ::mosek::fusion::Expression > _3632,int _3633);
void _initialize(int _3627,int _3628,std::shared_ptr< monty::ndarray< int,1 > > _3629,std::shared_ptr< monty::ndarray< int,1 > > _3630,std::shared_ptr< monty::ndarray< double,1 > > _3631,monty::rc_ptr< ::mosek::fusion::Expression > _3632,int _3633);
static ExprMulRight::t _new_ExprMulRight(int _3634,int _3635,std::shared_ptr< monty::ndarray< int,1 > > _3636,std::shared_ptr< monty::ndarray< int,1 > > _3637,std::shared_ptr< monty::ndarray< double,1 > > _3638,monty::rc_ptr< ::mosek::fusion::Expression > _3639);
void _initialize(int _3634,int _3635,std::shared_ptr< monty::ndarray< int,1 > > _3636,std::shared_ptr< monty::ndarray< int,1 > > _3637,std::shared_ptr< monty::ndarray< double,1 > > _3638,monty::rc_ptr< ::mosek::fusion::Expression > _3639);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _3640,monty::rc_ptr< ::mosek::fusion::WorkStack > _3641,monty::rc_ptr< ::mosek::fusion::WorkStack > _3642) ;
static  std::shared_ptr< monty::ndarray< int,1 > > computeshape(int _3791,std::shared_ptr< monty::ndarray< int,1 > > _3792);
static  int validate(int _3793,int _3794,std::shared_ptr< monty::ndarray< int,1 > > _3795,std::shared_ptr< monty::ndarray< int,1 > > _3796,std::shared_ptr< monty::ndarray< double,1 > > _3797,monty::rc_ptr< ::mosek::fusion::Expression > _3798);
virtual /* override */ std::string toString() ;
}; // struct ExprMulRight;

struct p_ExprMulLeft : public ::mosek::fusion::p_BaseExpression
{
ExprMulLeft * _pubthis;
static mosek::fusion::p_ExprMulLeft* _get_impl(mosek::fusion::ExprMulLeft * _inst){ return static_cast< mosek::fusion::p_ExprMulLeft* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprMulLeft * _get_impl(mosek::fusion::ExprMulLeft::t _inst) { return _get_impl(_inst.get()); }
p_ExprMulLeft(ExprMulLeft * _pubthis);
virtual ~p_ExprMulLeft() { /* std::cout << "~p_ExprMulLeft" << std::endl;*/ };
monty::rc_ptr< ::mosek::fusion::Expression > expr{};std::shared_ptr< monty::ndarray< double,1 > > mval{};std::shared_ptr< monty::ndarray< int,1 > > msubj{};std::shared_ptr< monty::ndarray< int,1 > > msubi{};int mdim1{};int mdim0{};virtual void destroy();
static ExprMulLeft::t _new_ExprMulLeft(int _3807,int _3808,std::shared_ptr< monty::ndarray< int,1 > > _3809,std::shared_ptr< monty::ndarray< int,1 > > _3810,std::shared_ptr< monty::ndarray< double,1 > > _3811,monty::rc_ptr< ::mosek::fusion::Expression > _3812,int _3813);
void _initialize(int _3807,int _3808,std::shared_ptr< monty::ndarray< int,1 > > _3809,std::shared_ptr< monty::ndarray< int,1 > > _3810,std::shared_ptr< monty::ndarray< double,1 > > _3811,monty::rc_ptr< ::mosek::fusion::Expression > _3812,int _3813);
static ExprMulLeft::t _new_ExprMulLeft(int _3814,int _3815,std::shared_ptr< monty::ndarray< int,1 > > _3816,std::shared_ptr< monty::ndarray< int,1 > > _3817,std::shared_ptr< monty::ndarray< double,1 > > _3818,monty::rc_ptr< ::mosek::fusion::Expression > _3819);
void _initialize(int _3814,int _3815,std::shared_ptr< monty::ndarray< int,1 > > _3816,std::shared_ptr< monty::ndarray< int,1 > > _3817,std::shared_ptr< monty::ndarray< double,1 > > _3818,monty::rc_ptr< ::mosek::fusion::Expression > _3819);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _3820,monty::rc_ptr< ::mosek::fusion::WorkStack > _3821,monty::rc_ptr< ::mosek::fusion::WorkStack > _3822) ;
static  std::shared_ptr< monty::ndarray< int,1 > > computeshape(int _3920,int _3921,std::shared_ptr< monty::ndarray< int,1 > > _3922);
static  int validate(int _3923,int _3924,std::shared_ptr< monty::ndarray< int,1 > > _3925,std::shared_ptr< monty::ndarray< int,1 > > _3926,std::shared_ptr< monty::ndarray< double,1 > > _3927,monty::rc_ptr< ::mosek::fusion::Expression > _3928);
virtual /* override */ std::string toString() ;
}; // struct ExprMulLeft;

struct p_ExprMulVar : public ::mosek::fusion::p_BaseExpression
{
ExprMulVar * _pubthis;
static mosek::fusion::p_ExprMulVar* _get_impl(mosek::fusion::ExprMulVar * _inst){ return static_cast< mosek::fusion::p_ExprMulVar* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprMulVar * _get_impl(mosek::fusion::ExprMulVar::t _inst) { return _get_impl(_inst.get()); }
p_ExprMulVar(ExprMulVar * _pubthis);
virtual ~p_ExprMulVar() { /* std::cout << "~p_ExprMulVar" << std::endl;*/ };
bool left{};monty::rc_ptr< ::mosek::fusion::Variable > x{};std::shared_ptr< monty::ndarray< double,1 > > mcof{};std::shared_ptr< monty::ndarray< int,1 > > msubj{};std::shared_ptr< monty::ndarray< int,1 > > msubi{};int mdimj{};int mdimi{};virtual void destroy();
static ExprMulVar::t _new_ExprMulVar(bool _3936,int _3937,int _3938,std::shared_ptr< monty::ndarray< int,1 > > _3939,std::shared_ptr< monty::ndarray< int,1 > > _3940,std::shared_ptr< monty::ndarray< double,1 > > _3941,monty::rc_ptr< ::mosek::fusion::Variable > _3942);
void _initialize(bool _3936,int _3937,int _3938,std::shared_ptr< monty::ndarray< int,1 > > _3939,std::shared_ptr< monty::ndarray< int,1 > > _3940,std::shared_ptr< monty::ndarray< double,1 > > _3941,monty::rc_ptr< ::mosek::fusion::Variable > _3942);
static ExprMulVar::t _new_ExprMulVar(bool _3945,int _3946,int _3947,std::shared_ptr< monty::ndarray< int,1 > > _3948,std::shared_ptr< monty::ndarray< int,1 > > _3949,std::shared_ptr< monty::ndarray< double,1 > > _3950,monty::rc_ptr< ::mosek::fusion::Variable > _3951,int _3952);
void _initialize(bool _3945,int _3946,int _3947,std::shared_ptr< monty::ndarray< int,1 > > _3948,std::shared_ptr< monty::ndarray< int,1 > > _3949,std::shared_ptr< monty::ndarray< double,1 > > _3950,monty::rc_ptr< ::mosek::fusion::Variable > _3951,int _3952);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _3953,monty::rc_ptr< ::mosek::fusion::WorkStack > _3954,monty::rc_ptr< ::mosek::fusion::WorkStack > _3955) ;
virtual void eval_right(monty::rc_ptr< ::mosek::fusion::WorkStack > _3956,monty::rc_ptr< ::mosek::fusion::WorkStack > _3957,monty::rc_ptr< ::mosek::fusion::WorkStack > _3958) ;
virtual void eval_left(monty::rc_ptr< ::mosek::fusion::WorkStack > _4064,monty::rc_ptr< ::mosek::fusion::WorkStack > _4065,monty::rc_ptr< ::mosek::fusion::WorkStack > _4066) ;
virtual void validate(int _4141,int _4142,std::shared_ptr< monty::ndarray< int,1 > > _4143,std::shared_ptr< monty::ndarray< int,1 > > _4144,std::shared_ptr< monty::ndarray< double,1 > > _4145) ;
static  std::shared_ptr< monty::ndarray< int,1 > > resshape(int _4149,int _4150,std::shared_ptr< monty::ndarray< int,1 > > _4151,bool _4152);
virtual /* override */ std::string toString() ;
}; // struct ExprMulVar;

struct p_ExprMulScalarVar : public ::mosek::fusion::p_BaseExpression
{
ExprMulScalarVar * _pubthis;
static mosek::fusion::p_ExprMulScalarVar* _get_impl(mosek::fusion::ExprMulScalarVar * _inst){ return static_cast< mosek::fusion::p_ExprMulScalarVar* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprMulScalarVar * _get_impl(mosek::fusion::ExprMulScalarVar::t _inst) { return _get_impl(_inst.get()); }
p_ExprMulScalarVar(ExprMulScalarVar * _pubthis);
virtual ~p_ExprMulScalarVar() { /* std::cout << "~p_ExprMulScalarVar" << std::endl;*/ };
monty::rc_ptr< ::mosek::fusion::Variable > x{};std::shared_ptr< monty::ndarray< double,1 > > mcof{};std::shared_ptr< monty::ndarray< int,1 > > msubj{};std::shared_ptr< monty::ndarray< int,1 > > msubi{};int mdimj{};int mdimi{};virtual void destroy();
static ExprMulScalarVar::t _new_ExprMulScalarVar(int _4153,int _4154,std::shared_ptr< monty::ndarray< int,1 > > _4155,std::shared_ptr< monty::ndarray< int,1 > > _4156,std::shared_ptr< monty::ndarray< double,1 > > _4157,monty::rc_ptr< ::mosek::fusion::Variable > _4158);
void _initialize(int _4153,int _4154,std::shared_ptr< monty::ndarray< int,1 > > _4155,std::shared_ptr< monty::ndarray< int,1 > > _4156,std::shared_ptr< monty::ndarray< double,1 > > _4157,monty::rc_ptr< ::mosek::fusion::Variable > _4158);
static ExprMulScalarVar::t _new_ExprMulScalarVar(int _4163,int _4164,std::shared_ptr< monty::ndarray< int,1 > > _4165,std::shared_ptr< monty::ndarray< int,1 > > _4166,std::shared_ptr< monty::ndarray< double,1 > > _4167,monty::rc_ptr< ::mosek::fusion::Variable > _4168,int _4169);
void _initialize(int _4163,int _4164,std::shared_ptr< monty::ndarray< int,1 > > _4165,std::shared_ptr< monty::ndarray< int,1 > > _4166,std::shared_ptr< monty::ndarray< double,1 > > _4167,monty::rc_ptr< ::mosek::fusion::Variable > _4168,int _4169);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _4170,monty::rc_ptr< ::mosek::fusion::WorkStack > _4171,monty::rc_ptr< ::mosek::fusion::WorkStack > _4172) ;
virtual /* override */ std::string toString() ;
}; // struct ExprMulScalarVar;

struct p_ExprMulVarScalarConst : public ::mosek::fusion::p_BaseExpression
{
ExprMulVarScalarConst * _pubthis;
static mosek::fusion::p_ExprMulVarScalarConst* _get_impl(mosek::fusion::ExprMulVarScalarConst * _inst){ return static_cast< mosek::fusion::p_ExprMulVarScalarConst* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprMulVarScalarConst * _get_impl(mosek::fusion::ExprMulVarScalarConst::t _inst) { return _get_impl(_inst.get()); }
p_ExprMulVarScalarConst(ExprMulVarScalarConst * _pubthis);
virtual ~p_ExprMulVarScalarConst() { /* std::cout << "~p_ExprMulVarScalarConst" << std::endl;*/ };
double c{};monty::rc_ptr< ::mosek::fusion::Variable > x{};virtual void destroy();
static ExprMulVarScalarConst::t _new_ExprMulVarScalarConst(monty::rc_ptr< ::mosek::fusion::Variable > _4191,double _4192);
void _initialize(monty::rc_ptr< ::mosek::fusion::Variable > _4191,double _4192);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _4193,monty::rc_ptr< ::mosek::fusion::WorkStack > _4194,monty::rc_ptr< ::mosek::fusion::WorkStack > _4195) ;
virtual /* override */ std::string toString() ;
}; // struct ExprMulVarScalarConst;

struct p_ExprAdd : public ::mosek::fusion::p_BaseExpression
{
ExprAdd * _pubthis;
static mosek::fusion::p_ExprAdd* _get_impl(mosek::fusion::ExprAdd * _inst){ return static_cast< mosek::fusion::p_ExprAdd* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprAdd * _get_impl(mosek::fusion::ExprAdd::t _inst) { return _get_impl(_inst.get()); }
p_ExprAdd(ExprAdd * _pubthis);
virtual ~p_ExprAdd() { /* std::cout << "~p_ExprAdd" << std::endl;*/ };
double m2{};double m1{};monty::rc_ptr< ::mosek::fusion::Expression > e2{};monty::rc_ptr< ::mosek::fusion::Expression > e1{};virtual void destroy();
static ExprAdd::t _new_ExprAdd(monty::rc_ptr< ::mosek::fusion::Expression > _4213,monty::rc_ptr< ::mosek::fusion::Expression > _4214,double _4215,double _4216);
void _initialize(monty::rc_ptr< ::mosek::fusion::Expression > _4213,monty::rc_ptr< ::mosek::fusion::Expression > _4214,double _4215,double _4216);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _4218,monty::rc_ptr< ::mosek::fusion::WorkStack > _4219,monty::rc_ptr< ::mosek::fusion::WorkStack > _4220) ;
virtual /* override */ std::string toString() ;
}; // struct ExprAdd;

struct p_ExprWSum : public ::mosek::fusion::p_BaseExpression
{
ExprWSum * _pubthis;
static mosek::fusion::p_ExprWSum* _get_impl(mosek::fusion::ExprWSum * _inst){ return static_cast< mosek::fusion::p_ExprWSum* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprWSum * _get_impl(mosek::fusion::ExprWSum::t _inst) { return _get_impl(_inst.get()); }
p_ExprWSum(ExprWSum * _pubthis);
virtual ~p_ExprWSum() { /* std::cout << "~p_ExprWSum" << std::endl;*/ };
std::shared_ptr< monty::ndarray< double,1 > > w{};std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Expression >,1 > > es{};virtual void destroy();
static ExprWSum::t _new_ExprWSum(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Expression >,1 > > _4330,std::shared_ptr< monty::ndarray< double,1 > > _4331);
void _initialize(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Expression >,1 > > _4330,std::shared_ptr< monty::ndarray< double,1 > > _4331);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _4338,monty::rc_ptr< ::mosek::fusion::WorkStack > _4339,monty::rc_ptr< ::mosek::fusion::WorkStack > _4340) ;
virtual /* override */ std::string toString() ;
}; // struct ExprWSum;

struct p_ExprSumReduce : public ::mosek::fusion::p_BaseExpression
{
ExprSumReduce * _pubthis;
static mosek::fusion::p_ExprSumReduce* _get_impl(mosek::fusion::ExprSumReduce * _inst){ return static_cast< mosek::fusion::p_ExprSumReduce* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprSumReduce * _get_impl(mosek::fusion::ExprSumReduce::t _inst) { return _get_impl(_inst.get()); }
p_ExprSumReduce(ExprSumReduce * _pubthis);
virtual ~p_ExprSumReduce() { /* std::cout << "~p_ExprSumReduce" << std::endl;*/ };
int dim{};monty::rc_ptr< ::mosek::fusion::Expression > expr{};virtual void destroy();
static ExprSumReduce::t _new_ExprSumReduce(int _4407,monty::rc_ptr< ::mosek::fusion::Expression > _4408);
void _initialize(int _4407,monty::rc_ptr< ::mosek::fusion::Expression > _4408);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _4410,monty::rc_ptr< ::mosek::fusion::WorkStack > _4411,monty::rc_ptr< ::mosek::fusion::WorkStack > _4412) ;
static  std::shared_ptr< monty::ndarray< int,1 > > computeShape(int _4516,std::shared_ptr< monty::ndarray< int,1 > > _4517);
virtual /* override */ std::string toString() ;
}; // struct ExprSumReduce;

struct p_ExprDenseTril : public ::mosek::fusion::p_BaseExpression
{
ExprDenseTril * _pubthis;
static mosek::fusion::p_ExprDenseTril* _get_impl(mosek::fusion::ExprDenseTril * _inst){ return static_cast< mosek::fusion::p_ExprDenseTril* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprDenseTril * _get_impl(mosek::fusion::ExprDenseTril::t _inst) { return _get_impl(_inst.get()); }
p_ExprDenseTril(ExprDenseTril * _pubthis);
virtual ~p_ExprDenseTril() { /* std::cout << "~p_ExprDenseTril" << std::endl;*/ };
int dim1{};int dim0{};monty::rc_ptr< ::mosek::fusion::Expression > expr{};virtual void destroy();
static ExprDenseTril::t _new_ExprDenseTril(int _4521,int _4522,monty::rc_ptr< ::mosek::fusion::Expression > _4523,int _4524);
void _initialize(int _4521,int _4522,monty::rc_ptr< ::mosek::fusion::Expression > _4523,int _4524);
static ExprDenseTril::t _new_ExprDenseTril(int _4525,int _4526,monty::rc_ptr< ::mosek::fusion::Expression > _4527);
void _initialize(int _4525,int _4526,monty::rc_ptr< ::mosek::fusion::Expression > _4527);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _4529,monty::rc_ptr< ::mosek::fusion::WorkStack > _4530,monty::rc_ptr< ::mosek::fusion::WorkStack > _4531) ;
}; // struct ExprDenseTril;

struct p_ExprDense : public ::mosek::fusion::p_BaseExpression
{
ExprDense * _pubthis;
static mosek::fusion::p_ExprDense* _get_impl(mosek::fusion::ExprDense * _inst){ return static_cast< mosek::fusion::p_ExprDense* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprDense * _get_impl(mosek::fusion::ExprDense::t _inst) { return _get_impl(_inst.get()); }
p_ExprDense(ExprDense * _pubthis);
virtual ~p_ExprDense() { /* std::cout << "~p_ExprDense" << std::endl;*/ };
monty::rc_ptr< ::mosek::fusion::Expression > expr{};virtual void destroy();
static ExprDense::t _new_ExprDense(monty::rc_ptr< ::mosek::fusion::Expression > _4604);
void _initialize(monty::rc_ptr< ::mosek::fusion::Expression > _4604);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _4605,monty::rc_ptr< ::mosek::fusion::WorkStack > _4606,monty::rc_ptr< ::mosek::fusion::WorkStack > _4607) ;
virtual /* override */ std::string toString() ;
}; // struct ExprDense;

struct p_ExprSymmetrize : public ::mosek::fusion::p_BaseExpression
{
ExprSymmetrize * _pubthis;
static mosek::fusion::p_ExprSymmetrize* _get_impl(mosek::fusion::ExprSymmetrize * _inst){ return static_cast< mosek::fusion::p_ExprSymmetrize* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprSymmetrize * _get_impl(mosek::fusion::ExprSymmetrize::t _inst) { return _get_impl(_inst.get()); }
p_ExprSymmetrize(ExprSymmetrize * _pubthis);
virtual ~p_ExprSymmetrize() { /* std::cout << "~p_ExprSymmetrize" << std::endl;*/ };
int dim1{};int dim0{};monty::rc_ptr< ::mosek::fusion::Expression > expr{};virtual void destroy();
static ExprSymmetrize::t _new_ExprSymmetrize(int _4630,int _4631,monty::rc_ptr< ::mosek::fusion::Expression > _4632,int _4633);
void _initialize(int _4630,int _4631,monty::rc_ptr< ::mosek::fusion::Expression > _4632,int _4633);
static ExprSymmetrize::t _new_ExprSymmetrize(int _4634,int _4635,monty::rc_ptr< ::mosek::fusion::Expression > _4636);
void _initialize(int _4634,int _4635,monty::rc_ptr< ::mosek::fusion::Expression > _4636);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _4638,monty::rc_ptr< ::mosek::fusion::WorkStack > _4639,monty::rc_ptr< ::mosek::fusion::WorkStack > _4640) ;
}; // struct ExprSymmetrize;

struct p_ExprCompress : public ::mosek::fusion::p_BaseExpression
{
ExprCompress * _pubthis;
static mosek::fusion::p_ExprCompress* _get_impl(mosek::fusion::ExprCompress * _inst){ return static_cast< mosek::fusion::p_ExprCompress* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprCompress * _get_impl(mosek::fusion::ExprCompress::t _inst) { return _get_impl(_inst.get()); }
p_ExprCompress(ExprCompress * _pubthis);
virtual ~p_ExprCompress() { /* std::cout << "~p_ExprCompress" << std::endl;*/ };
double eps{};monty::rc_ptr< ::mosek::fusion::Expression > expr{};virtual void destroy();
static ExprCompress::t _new_ExprCompress(monty::rc_ptr< ::mosek::fusion::Expression > _4739);
void _initialize(monty::rc_ptr< ::mosek::fusion::Expression > _4739);
static ExprCompress::t _new_ExprCompress(monty::rc_ptr< ::mosek::fusion::Expression > _4740,double _4741);
void _initialize(monty::rc_ptr< ::mosek::fusion::Expression > _4740,double _4741);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _4742,monty::rc_ptr< ::mosek::fusion::WorkStack > _4743,monty::rc_ptr< ::mosek::fusion::WorkStack > _4744) ;
static  void arg_sort(monty::rc_ptr< ::mosek::fusion::WorkStack > _4810,monty::rc_ptr< ::mosek::fusion::WorkStack > _4811,int _4812,int _4813,int _4814,int _4815,int _4816);
static  void merge_sort(int _4852,int _4853,int _4854,int _4855,int _4856,int _4857,std::shared_ptr< monty::ndarray< int,1 > > _4858,std::shared_ptr< monty::ndarray< long long,1 > > _4859);
virtual /* override */ std::string toString() ;
}; // struct ExprCompress;

struct p_ExprCondense : public ::mosek::fusion::p_BaseExpression
{
ExprCondense * _pubthis;
static mosek::fusion::p_ExprCondense* _get_impl(mosek::fusion::ExprCondense * _inst){ return static_cast< mosek::fusion::p_ExprCondense* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprCondense * _get_impl(mosek::fusion::ExprCondense::t _inst) { return _get_impl(_inst.get()); }
p_ExprCondense(ExprCondense * _pubthis);
virtual ~p_ExprCondense() { /* std::cout << "~p_ExprCondense" << std::endl;*/ };
monty::rc_ptr< ::mosek::fusion::Expression > expr{};virtual void destroy();
static ExprCondense::t _new_ExprCondense(monty::rc_ptr< ::mosek::fusion::Expression > _4882);
void _initialize(monty::rc_ptr< ::mosek::fusion::Expression > _4882);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _4883,monty::rc_ptr< ::mosek::fusion::WorkStack > _4884,monty::rc_ptr< ::mosek::fusion::WorkStack > _4885) ;
}; // struct ExprCondense;

struct p_ExprFromVar : public ::mosek::fusion::p_BaseExpression
{
ExprFromVar * _pubthis;
static mosek::fusion::p_ExprFromVar* _get_impl(mosek::fusion::ExprFromVar * _inst){ return static_cast< mosek::fusion::p_ExprFromVar* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprFromVar * _get_impl(mosek::fusion::ExprFromVar::t _inst) { return _get_impl(_inst.get()); }
p_ExprFromVar(ExprFromVar * _pubthis);
virtual ~p_ExprFromVar() { /* std::cout << "~p_ExprFromVar" << std::endl;*/ };
monty::rc_ptr< ::mosek::fusion::Variable > x{};virtual void destroy();
static ExprFromVar::t _new_ExprFromVar(monty::rc_ptr< ::mosek::fusion::Variable > _4892);
void _initialize(monty::rc_ptr< ::mosek::fusion::Variable > _4892);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _4893,monty::rc_ptr< ::mosek::fusion::WorkStack > _4894,monty::rc_ptr< ::mosek::fusion::WorkStack > _4895) ;
virtual /* override */ std::string toString() ;
}; // struct ExprFromVar;

struct p_ExprReshape : public ::mosek::fusion::p_BaseExpression
{
ExprReshape * _pubthis;
static mosek::fusion::p_ExprReshape* _get_impl(mosek::fusion::ExprReshape * _inst){ return static_cast< mosek::fusion::p_ExprReshape* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprReshape * _get_impl(mosek::fusion::ExprReshape::t _inst) { return _get_impl(_inst.get()); }
p_ExprReshape(ExprReshape * _pubthis);
virtual ~p_ExprReshape() { /* std::cout << "~p_ExprReshape" << std::endl;*/ };
monty::rc_ptr< ::mosek::fusion::Expression > e{};virtual void destroy();
static ExprReshape::t _new_ExprReshape(std::shared_ptr< monty::ndarray< int,1 > > _4913,monty::rc_ptr< ::mosek::fusion::Expression > _4914);
void _initialize(std::shared_ptr< monty::ndarray< int,1 > > _4913,monty::rc_ptr< ::mosek::fusion::Expression > _4914);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _4916,monty::rc_ptr< ::mosek::fusion::WorkStack > _4917,monty::rc_ptr< ::mosek::fusion::WorkStack > _4918) ;
virtual /* override */ std::string toString() ;
}; // struct ExprReshape;

struct p_Expr : public ::mosek::fusion::p_BaseExpression
{
Expr * _pubthis;
static mosek::fusion::p_Expr* _get_impl(mosek::fusion::Expr * _inst){ return static_cast< mosek::fusion::p_Expr* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_Expr * _get_impl(mosek::fusion::Expr::t _inst) { return _get_impl(_inst.get()); }
p_Expr(Expr * _pubthis);
virtual ~p_Expr() { /* std::cout << "~p_Expr" << std::endl;*/ };
std::shared_ptr< monty::ndarray< long long,1 > > inst{};std::shared_ptr< monty::ndarray< double,1 > > cof_v{};std::shared_ptr< monty::ndarray< long long,1 > > subj{};std::shared_ptr< monty::ndarray< long long,1 > > ptrb{};std::shared_ptr< monty::ndarray< double,1 > > bfix{};std::shared_ptr< monty::ndarray< int,1 > > shape{};virtual void destroy();
static Expr::t _new_Expr(std::shared_ptr< monty::ndarray< long long,1 > > _5050,std::shared_ptr< monty::ndarray< long long,1 > > _5051,std::shared_ptr< monty::ndarray< double,1 > > _5052,std::shared_ptr< monty::ndarray< double,1 > > _5053,std::shared_ptr< monty::ndarray< int,1 > > _5054,std::shared_ptr< monty::ndarray< long long,1 > > _5055);
void _initialize(std::shared_ptr< monty::ndarray< long long,1 > > _5050,std::shared_ptr< monty::ndarray< long long,1 > > _5051,std::shared_ptr< monty::ndarray< double,1 > > _5052,std::shared_ptr< monty::ndarray< double,1 > > _5053,std::shared_ptr< monty::ndarray< int,1 > > _5054,std::shared_ptr< monty::ndarray< long long,1 > > _5055);
static Expr::t _new_Expr(std::shared_ptr< monty::ndarray< long long,1 > > _5066,std::shared_ptr< monty::ndarray< long long,1 > > _5067,std::shared_ptr< monty::ndarray< double,1 > > _5068,std::shared_ptr< monty::ndarray< double,1 > > _5069,std::shared_ptr< monty::ndarray< int,1 > > _5070,std::shared_ptr< monty::ndarray< long long,1 > > _5071,int _5072);
void _initialize(std::shared_ptr< monty::ndarray< long long,1 > > _5066,std::shared_ptr< monty::ndarray< long long,1 > > _5067,std::shared_ptr< monty::ndarray< double,1 > > _5068,std::shared_ptr< monty::ndarray< double,1 > > _5069,std::shared_ptr< monty::ndarray< int,1 > > _5070,std::shared_ptr< monty::ndarray< long long,1 > > _5071,int _5072);
static Expr::t _new_Expr(monty::rc_ptr< ::mosek::fusion::Expression > _5073);
void _initialize(monty::rc_ptr< ::mosek::fusion::Expression > _5073);
virtual long long prod(std::shared_ptr< monty::ndarray< int,1 > > _5098) ;
static  std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Variable >,1 > > varstack(std::shared_ptr< monty::ndarray< std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Variable >,1 > >,1 > > _5101);
static  std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Variable >,1 > > varstack(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Variable >,1 > > _5104,std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Variable >,1 > > _5105);
static  monty::rc_ptr< ::mosek::fusion::Expression > condense(monty::rc_ptr< ::mosek::fusion::Expression > _5109);
static  monty::rc_ptr< ::mosek::fusion::Expression > flatten(monty::rc_ptr< ::mosek::fusion::Expression > _5110);
static  monty::rc_ptr< ::mosek::fusion::Expression > reshape(monty::rc_ptr< ::mosek::fusion::Expression > _5111,int _5112,int _5113);
static  monty::rc_ptr< ::mosek::fusion::Expression > reshape(monty::rc_ptr< ::mosek::fusion::Expression > _5114,int _5115);
static  monty::rc_ptr< ::mosek::fusion::Expression > reshape(monty::rc_ptr< ::mosek::fusion::Expression > _5116,std::shared_ptr< monty::ndarray< int,1 > > _5117);
virtual long long size() ;
virtual /* override */ monty::rc_ptr< ::mosek::fusion::FlatExpr > __mosek_2fusion_2Expr__eval() ;
virtual monty::rc_ptr< ::mosek::fusion::FlatExpr > __mosek_2fusion_2BaseExpression__eval() { return __mosek_2fusion_2Expr__eval(); }
static  monty::rc_ptr< ::mosek::fusion::Expression > zeros(std::shared_ptr< monty::ndarray< int,1 > > _5120);
static  monty::rc_ptr< ::mosek::fusion::Expression > zeros(int _5121);
static  monty::rc_ptr< ::mosek::fusion::Expression > ones();
static  monty::rc_ptr< ::mosek::fusion::Expression > ones(std::shared_ptr< monty::ndarray< int,1 > > _5122,std::shared_ptr< monty::ndarray< int,2 > > _5123);
static  monty::rc_ptr< ::mosek::fusion::Expression > ones(std::shared_ptr< monty::ndarray< int,1 > > _5124);
static  monty::rc_ptr< ::mosek::fusion::Expression > ones(int _5125);
static  monty::rc_ptr< ::mosek::fusion::Expression > constTerm(monty::rc_ptr< ::mosek::fusion::NDSparseArray > _5126);
static  monty::rc_ptr< ::mosek::fusion::Expression > constTerm(monty::rc_ptr< ::mosek::fusion::Matrix > _5127);
static  monty::rc_ptr< ::mosek::fusion::Expression > constTerm(double _5136);
static  monty::rc_ptr< ::mosek::fusion::Expression > constTerm(std::shared_ptr< monty::ndarray< int,1 > > _5137,std::shared_ptr< monty::ndarray< int,2 > > _5138,double _5139);
static  monty::rc_ptr< ::mosek::fusion::Expression > constTerm(std::shared_ptr< monty::ndarray< int,1 > > _5147,std::shared_ptr< monty::ndarray< int,2 > > _5148,std::shared_ptr< monty::ndarray< double,1 > > _5149);
static  monty::rc_ptr< ::mosek::fusion::Expression > constTerm(std::shared_ptr< monty::ndarray< int,1 > > _5157,double _5158);
static  monty::rc_ptr< ::mosek::fusion::Expression > constTerm(int _5159,double _5160);
static  monty::rc_ptr< ::mosek::fusion::Expression > constTerm(std::shared_ptr< monty::ndarray< double,2 > > _5162);
static  monty::rc_ptr< ::mosek::fusion::Expression > constTerm(std::shared_ptr< monty::ndarray< double,1 > > _5165);
virtual long long numNonzeros() ;
static  monty::rc_ptr< ::mosek::fusion::Expression > sum(monty::rc_ptr< ::mosek::fusion::Expression > _5166,int _5167);
static  monty::rc_ptr< ::mosek::fusion::Expression > sum(monty::rc_ptr< ::mosek::fusion::Expression > _5168);
static  monty::rc_ptr< ::mosek::fusion::Expression > neg(monty::rc_ptr< ::mosek::fusion::Expression > _5169);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulDiag(bool _5170,monty::rc_ptr< ::mosek::fusion::Matrix > _5171,monty::rc_ptr< ::mosek::fusion::Expression > _5172);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulDiag(monty::rc_ptr< ::mosek::fusion::Variable > _5179,monty::rc_ptr< ::mosek::fusion::Matrix > _5180);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulDiag(monty::rc_ptr< ::mosek::fusion::Matrix > _5181,monty::rc_ptr< ::mosek::fusion::Variable > _5182);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulDiag(monty::rc_ptr< ::mosek::fusion::Expression > _5183,monty::rc_ptr< ::mosek::fusion::Matrix > _5184);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulDiag(monty::rc_ptr< ::mosek::fusion::Matrix > _5185,monty::rc_ptr< ::mosek::fusion::Expression > _5186);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulDiag(monty::rc_ptr< ::mosek::fusion::Variable > _5187,std::shared_ptr< monty::ndarray< double,2 > > _5188);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulDiag(monty::rc_ptr< ::mosek::fusion::Expression > _5195,std::shared_ptr< monty::ndarray< double,2 > > _5196);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulDiag(std::shared_ptr< monty::ndarray< double,2 > > _5203,monty::rc_ptr< ::mosek::fusion::Variable > _5204);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulDiag(std::shared_ptr< monty::ndarray< double,2 > > _5211,monty::rc_ptr< ::mosek::fusion::Expression > _5212);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulElm_(monty::rc_ptr< ::mosek::fusion::Matrix > _5219,monty::rc_ptr< ::mosek::fusion::Expression > _5220);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulElm_(std::shared_ptr< monty::ndarray< double,1 > > _5229,monty::rc_ptr< ::mosek::fusion::Expression > _5230);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulElm_(monty::rc_ptr< ::mosek::fusion::NDSparseArray > _5232,monty::rc_ptr< ::mosek::fusion::Expression > _5233);
static  monty::rc_ptr< ::mosek::fusion::Expression > mul(monty::rc_ptr< ::mosek::fusion::Expression > _5236,double _5237);
static  monty::rc_ptr< ::mosek::fusion::Expression > mul(double _5238,monty::rc_ptr< ::mosek::fusion::Expression > _5239);
static  monty::rc_ptr< ::mosek::fusion::Expression > mul(monty::rc_ptr< ::mosek::fusion::Expression > _5240,std::shared_ptr< monty::ndarray< double,1 > > _5241);
static  monty::rc_ptr< ::mosek::fusion::Expression > mul(std::shared_ptr< monty::ndarray< double,1 > > _5242,monty::rc_ptr< ::mosek::fusion::Expression > _5243);
static  monty::rc_ptr< ::mosek::fusion::Expression > mul(monty::rc_ptr< ::mosek::fusion::Expression > _5244,std::shared_ptr< monty::ndarray< double,2 > > _5245);
static  monty::rc_ptr< ::mosek::fusion::Expression > mul(std::shared_ptr< monty::ndarray< double,2 > > _5246,monty::rc_ptr< ::mosek::fusion::Expression > _5247);
static  monty::rc_ptr< ::mosek::fusion::Expression > mul(monty::rc_ptr< ::mosek::fusion::Expression > _5248,monty::rc_ptr< ::mosek::fusion::Matrix > _5249);
static  monty::rc_ptr< ::mosek::fusion::Expression > mul(monty::rc_ptr< ::mosek::fusion::Matrix > _5250,monty::rc_ptr< ::mosek::fusion::Expression > _5251);
static  monty::rc_ptr< ::mosek::fusion::Expression > mul(bool _5252,std::shared_ptr< monty::ndarray< double,1 > > _5253,monty::rc_ptr< ::mosek::fusion::Expression > _5254);
static  monty::rc_ptr< ::mosek::fusion::Expression > mul(bool _5269,std::shared_ptr< monty::ndarray< double,2 > > _5270,monty::rc_ptr< ::mosek::fusion::Expression > _5271);
static  monty::rc_ptr< ::mosek::fusion::Expression > mul(bool _5286,monty::rc_ptr< ::mosek::fusion::Matrix > _5287,monty::rc_ptr< ::mosek::fusion::Expression > _5288);
static  monty::rc_ptr< ::mosek::fusion::Expression > mul(monty::rc_ptr< ::mosek::fusion::Variable > _5297,monty::rc_ptr< ::mosek::fusion::Matrix > _5298);
static  monty::rc_ptr< ::mosek::fusion::Expression > mul(monty::rc_ptr< ::mosek::fusion::Matrix > _5304,monty::rc_ptr< ::mosek::fusion::Variable > _5305);
static  monty::rc_ptr< ::mosek::fusion::Expression > mul(bool _5311,int _5312,int _5313,std::shared_ptr< monty::ndarray< int,1 > > _5314,std::shared_ptr< monty::ndarray< int,1 > > _5315,std::shared_ptr< monty::ndarray< double,1 > > _5316,monty::rc_ptr< ::mosek::fusion::Variable > _5317);
static  monty::rc_ptr< ::mosek::fusion::Expression > dot(monty::rc_ptr< ::mosek::fusion::Expression > _5319,monty::rc_ptr< ::mosek::fusion::Matrix > _5320);
static  monty::rc_ptr< ::mosek::fusion::Expression > dot(monty::rc_ptr< ::mosek::fusion::Expression > _5328,std::shared_ptr< monty::ndarray< double,2 > > _5329);
static  monty::rc_ptr< ::mosek::fusion::Expression > dot(monty::rc_ptr< ::mosek::fusion::Expression > _5333,monty::rc_ptr< ::mosek::fusion::NDSparseArray > _5334);
static  monty::rc_ptr< ::mosek::fusion::Expression > dot(monty::rc_ptr< ::mosek::fusion::Expression > _5335,std::shared_ptr< monty::ndarray< double,1 > > _5336);
static  monty::rc_ptr< ::mosek::fusion::Expression > dot(monty::rc_ptr< ::mosek::fusion::Matrix > _5341,monty::rc_ptr< ::mosek::fusion::Expression > _5342);
static  monty::rc_ptr< ::mosek::fusion::Expression > dot(monty::rc_ptr< ::mosek::fusion::NDSparseArray > _5343,monty::rc_ptr< ::mosek::fusion::Expression > _5344);
static  monty::rc_ptr< ::mosek::fusion::Expression > dot(std::shared_ptr< monty::ndarray< double,2 > > _5345,monty::rc_ptr< ::mosek::fusion::Expression > _5346);
static  monty::rc_ptr< ::mosek::fusion::Expression > dot(std::shared_ptr< monty::ndarray< double,1 > > _5347,monty::rc_ptr< ::mosek::fusion::Expression > _5348);
static  monty::rc_ptr< ::mosek::fusion::Expression > outer(std::shared_ptr< monty::ndarray< double,1 > > _5349,monty::rc_ptr< ::mosek::fusion::Expression > _5350);
static  monty::rc_ptr< ::mosek::fusion::Expression > outer(monty::rc_ptr< ::mosek::fusion::Expression > _5354,std::shared_ptr< monty::ndarray< double,1 > > _5355);
static  monty::rc_ptr< ::mosek::fusion::Expression > outer(monty::rc_ptr< ::mosek::fusion::Matrix > _5358,monty::rc_ptr< ::mosek::fusion::Variable > _5359);
static  monty::rc_ptr< ::mosek::fusion::Expression > outer(monty::rc_ptr< ::mosek::fusion::Variable > _5366,monty::rc_ptr< ::mosek::fusion::Matrix > _5367);
static  monty::rc_ptr< ::mosek::fusion::Expression > outer(std::shared_ptr< monty::ndarray< double,1 > > _5374,monty::rc_ptr< ::mosek::fusion::Variable > _5375);
static  monty::rc_ptr< ::mosek::fusion::Expression > outer(monty::rc_ptr< ::mosek::fusion::Variable > _5377,std::shared_ptr< monty::ndarray< double,1 > > _5378);
static  monty::rc_ptr< ::mosek::fusion::Expression > outer_(int _5380,std::shared_ptr< monty::ndarray< long long,1 > > _5381,std::shared_ptr< monty::ndarray< long long,1 > > _5382,std::shared_ptr< monty::ndarray< double,1 > > _5383,std::shared_ptr< monty::ndarray< double,1 > > _5384,std::shared_ptr< monty::ndarray< long long,1 > > _5385,std::shared_ptr< monty::ndarray< double,1 > > _5386,std::shared_ptr< monty::ndarray< int,1 > > _5387,int _5388,bool _5389);
static  monty::rc_ptr< ::mosek::fusion::Expression > outer_(monty::rc_ptr< ::mosek::fusion::Variable > _5419,int _5420,std::shared_ptr< monty::ndarray< double,1 > > _5421,std::shared_ptr< monty::ndarray< int,1 > > _5422,int _5423,bool _5424);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(std::shared_ptr< monty::ndarray< std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Expression >,1 > >,1 > > _5441);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(double _5447,double _5448,double _5449);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(double _5450,double _5451,monty::rc_ptr< ::mosek::fusion::Expression > _5452);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(double _5453,monty::rc_ptr< ::mosek::fusion::Expression > _5454,double _5455);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(double _5456,monty::rc_ptr< ::mosek::fusion::Expression > _5457,monty::rc_ptr< ::mosek::fusion::Expression > _5458);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(monty::rc_ptr< ::mosek::fusion::Expression > _5459,double _5460,double _5461);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(monty::rc_ptr< ::mosek::fusion::Expression > _5462,double _5463,monty::rc_ptr< ::mosek::fusion::Expression > _5464);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(monty::rc_ptr< ::mosek::fusion::Expression > _5465,monty::rc_ptr< ::mosek::fusion::Expression > _5466,double _5467);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(monty::rc_ptr< ::mosek::fusion::Expression > _5468,monty::rc_ptr< ::mosek::fusion::Expression > _5469,monty::rc_ptr< ::mosek::fusion::Expression > _5470);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(double _5471,monty::rc_ptr< ::mosek::fusion::Expression > _5472);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(monty::rc_ptr< ::mosek::fusion::Expression > _5473,double _5474);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(monty::rc_ptr< ::mosek::fusion::Expression > _5475,monty::rc_ptr< ::mosek::fusion::Expression > _5476);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Expression >,1 > > _5477);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(monty::rc_ptr< ::mosek::fusion::Expression > _5479,monty::rc_ptr< ::mosek::fusion::Expression > _5480,monty::rc_ptr< ::mosek::fusion::Expression > _5481);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(monty::rc_ptr< ::mosek::fusion::Expression > _5482,monty::rc_ptr< ::mosek::fusion::Expression > _5483,double _5484);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(monty::rc_ptr< ::mosek::fusion::Expression > _5485,double _5486,monty::rc_ptr< ::mosek::fusion::Expression > _5487);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(monty::rc_ptr< ::mosek::fusion::Expression > _5488,double _5489,double _5490);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(double _5491,monty::rc_ptr< ::mosek::fusion::Expression > _5492,monty::rc_ptr< ::mosek::fusion::Expression > _5493);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(double _5494,monty::rc_ptr< ::mosek::fusion::Expression > _5495,double _5496);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(double _5497,double _5498,monty::rc_ptr< ::mosek::fusion::Expression > _5499);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(double _5500,monty::rc_ptr< ::mosek::fusion::Expression > _5501);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(monty::rc_ptr< ::mosek::fusion::Expression > _5502,double _5503);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(monty::rc_ptr< ::mosek::fusion::Expression > _5504,monty::rc_ptr< ::mosek::fusion::Expression > _5505);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Expression >,1 > > _5506);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int _5508,monty::rc_ptr< ::mosek::fusion::Expression > _5509,monty::rc_ptr< ::mosek::fusion::Expression > _5510,monty::rc_ptr< ::mosek::fusion::Expression > _5511);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int _5512,monty::rc_ptr< ::mosek::fusion::Expression > _5513,monty::rc_ptr< ::mosek::fusion::Expression > _5514,double _5515);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int _5516,monty::rc_ptr< ::mosek::fusion::Expression > _5517,double _5518,monty::rc_ptr< ::mosek::fusion::Expression > _5519);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int _5520,monty::rc_ptr< ::mosek::fusion::Expression > _5521,double _5522,double _5523);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int _5524,double _5525,monty::rc_ptr< ::mosek::fusion::Expression > _5526,monty::rc_ptr< ::mosek::fusion::Expression > _5527);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int _5528,double _5529,monty::rc_ptr< ::mosek::fusion::Expression > _5530,double _5531);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int _5532,double _5533,double _5534,monty::rc_ptr< ::mosek::fusion::Expression > _5535);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int _5536,double _5537,monty::rc_ptr< ::mosek::fusion::Expression > _5538);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int _5539,monty::rc_ptr< ::mosek::fusion::Expression > _5540,double _5541);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int _5542,monty::rc_ptr< ::mosek::fusion::Expression > _5543,monty::rc_ptr< ::mosek::fusion::Expression > _5544);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int _5545,std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Expression >,1 > > _5546);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack_(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Expression >,1 > > _5547,int _5548);
static  std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Expression >,1 > > promote(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Expression >,1 > > _5549,int _5550);
static  monty::rc_ptr< ::mosek::fusion::Expression > repeat(monty::rc_ptr< ::mosek::fusion::Expression > _5563,int _5564,int _5565);
static  monty::rc_ptr< ::mosek::fusion::Expression > add(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Expression >,1 > > _5567);
static  monty::rc_ptr< ::mosek::fusion::Expression > add(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Variable >,1 > > _5569);
static  monty::rc_ptr< ::mosek::fusion::Expression > add_(monty::rc_ptr< ::mosek::fusion::Expression > _5602,double _5603,monty::rc_ptr< ::mosek::fusion::Expression > _5604,double _5605);
static  monty::rc_ptr< ::mosek::fusion::Expression > transpose(monty::rc_ptr< ::mosek::fusion::Expression > _5616);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulElm(monty::rc_ptr< ::mosek::fusion::Matrix > _5617,monty::rc_ptr< ::mosek::fusion::Expression > _5618);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulElm(monty::rc_ptr< ::mosek::fusion::NDSparseArray > _5619,monty::rc_ptr< ::mosek::fusion::Expression > _5620);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulElm(std::shared_ptr< monty::ndarray< double,2 > > _5621,monty::rc_ptr< ::mosek::fusion::Expression > _5622);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulElm(std::shared_ptr< monty::ndarray< double,1 > > _5623,monty::rc_ptr< ::mosek::fusion::Expression > _5624);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulElm(monty::rc_ptr< ::mosek::fusion::Expression > _5625,monty::rc_ptr< ::mosek::fusion::Matrix > _5626);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulElm(monty::rc_ptr< ::mosek::fusion::Expression > _5627,std::shared_ptr< monty::ndarray< double,2 > > _5628);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulElm(monty::rc_ptr< ::mosek::fusion::Expression > _5629,std::shared_ptr< monty::ndarray< double,1 > > _5630);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulElm(monty::rc_ptr< ::mosek::fusion::Expression > _5631,monty::rc_ptr< ::mosek::fusion::NDSparseArray > _5632);
static  monty::rc_ptr< ::mosek::fusion::Expression > sub(monty::rc_ptr< ::mosek::fusion::NDSparseArray > _5633,monty::rc_ptr< ::mosek::fusion::Expression > _5634);
static  monty::rc_ptr< ::mosek::fusion::Expression > sub(monty::rc_ptr< ::mosek::fusion::Expression > _5635,monty::rc_ptr< ::mosek::fusion::NDSparseArray > _5636);
static  monty::rc_ptr< ::mosek::fusion::Expression > sub(monty::rc_ptr< ::mosek::fusion::Matrix > _5637,monty::rc_ptr< ::mosek::fusion::Expression > _5638);
static  monty::rc_ptr< ::mosek::fusion::Expression > sub(monty::rc_ptr< ::mosek::fusion::Expression > _5639,monty::rc_ptr< ::mosek::fusion::Matrix > _5640);
static  monty::rc_ptr< ::mosek::fusion::Expression > sub(double _5641,monty::rc_ptr< ::mosek::fusion::Expression > _5642);
static  monty::rc_ptr< ::mosek::fusion::Expression > sub(monty::rc_ptr< ::mosek::fusion::Expression > _5643,double _5644);
static  monty::rc_ptr< ::mosek::fusion::Expression > sub(std::shared_ptr< monty::ndarray< double,2 > > _5645,monty::rc_ptr< ::mosek::fusion::Expression > _5646);
static  monty::rc_ptr< ::mosek::fusion::Expression > sub(std::shared_ptr< monty::ndarray< double,1 > > _5647,monty::rc_ptr< ::mosek::fusion::Expression > _5648);
static  monty::rc_ptr< ::mosek::fusion::Expression > sub(monty::rc_ptr< ::mosek::fusion::Expression > _5649,std::shared_ptr< monty::ndarray< double,2 > > _5650);
static  monty::rc_ptr< ::mosek::fusion::Expression > sub(monty::rc_ptr< ::mosek::fusion::Expression > _5651,std::shared_ptr< monty::ndarray< double,1 > > _5652);
static  monty::rc_ptr< ::mosek::fusion::Expression > sub(monty::rc_ptr< ::mosek::fusion::Expression > _5653,monty::rc_ptr< ::mosek::fusion::Expression > _5654);
static  monty::rc_ptr< ::mosek::fusion::Expression > add(monty::rc_ptr< ::mosek::fusion::NDSparseArray > _5655,monty::rc_ptr< ::mosek::fusion::Expression > _5656);
static  monty::rc_ptr< ::mosek::fusion::Expression > add(monty::rc_ptr< ::mosek::fusion::Expression > _5657,monty::rc_ptr< ::mosek::fusion::NDSparseArray > _5658);
static  monty::rc_ptr< ::mosek::fusion::Expression > add(monty::rc_ptr< ::mosek::fusion::Matrix > _5659,monty::rc_ptr< ::mosek::fusion::Expression > _5660);
static  monty::rc_ptr< ::mosek::fusion::Expression > add(monty::rc_ptr< ::mosek::fusion::Expression > _5661,monty::rc_ptr< ::mosek::fusion::Matrix > _5662);
static  monty::rc_ptr< ::mosek::fusion::Expression > add(double _5663,monty::rc_ptr< ::mosek::fusion::Expression > _5664);
static  monty::rc_ptr< ::mosek::fusion::Expression > add(monty::rc_ptr< ::mosek::fusion::Expression > _5665,double _5666);
static  monty::rc_ptr< ::mosek::fusion::Expression > add(std::shared_ptr< monty::ndarray< double,2 > > _5667,monty::rc_ptr< ::mosek::fusion::Expression > _5668);
static  monty::rc_ptr< ::mosek::fusion::Expression > add(std::shared_ptr< monty::ndarray< double,1 > > _5669,monty::rc_ptr< ::mosek::fusion::Expression > _5670);
static  monty::rc_ptr< ::mosek::fusion::Expression > add(monty::rc_ptr< ::mosek::fusion::Expression > _5671,std::shared_ptr< monty::ndarray< double,2 > > _5672);
static  monty::rc_ptr< ::mosek::fusion::Expression > add(monty::rc_ptr< ::mosek::fusion::Expression > _5673,std::shared_ptr< monty::ndarray< double,1 > > _5674);
static  monty::rc_ptr< ::mosek::fusion::Expression > add(monty::rc_ptr< ::mosek::fusion::Expression > _5675,monty::rc_ptr< ::mosek::fusion::Expression > _5676);
virtual /* override */ int getND() ;
virtual /* override */ std::shared_ptr< monty::ndarray< int,1 > > getShape() ;
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _5677,monty::rc_ptr< ::mosek::fusion::WorkStack > _5678,monty::rc_ptr< ::mosek::fusion::WorkStack > _5679) ;
static  void validateData(std::shared_ptr< monty::ndarray< long long,1 > > _5681,std::shared_ptr< monty::ndarray< long long,1 > > _5682,std::shared_ptr< monty::ndarray< double,1 > > _5683,std::shared_ptr< monty::ndarray< double,1 > > _5684,std::shared_ptr< monty::ndarray< int,1 > > _5685,std::shared_ptr< monty::ndarray< long long,1 > > _5686);
static  monty::rc_ptr< ::mosek::fusion::Model > extractModel(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Variable >,1 > > _5699);
}; // struct Expr;

struct p_WorkStack
{
WorkStack * _pubthis;
static mosek::fusion::p_WorkStack* _get_impl(mosek::fusion::WorkStack * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_WorkStack * _get_impl(mosek::fusion::WorkStack::t _inst) { return _get_impl(_inst.get()); }
p_WorkStack(WorkStack * _pubthis);
virtual ~p_WorkStack() { /* std::cout << "~p_WorkStack" << std::endl;*/ };
int cof_base{};int bfix_base{};int nidxs_base{};int sp_base{};int shape_base{};int ptr_base{};bool hassp{};int nelem{};int nnz{};int nd{};int pf64{};int pi64{};int pi32{};std::shared_ptr< monty::ndarray< double,1 > > f64{};std::shared_ptr< monty::ndarray< long long,1 > > i64{};std::shared_ptr< monty::ndarray< int,1 > > i32{};virtual void destroy();
static WorkStack::t _new_WorkStack();
void _initialize();
virtual bool peek_hassp() ;
virtual int peek_nnz() ;
virtual int peek_nelem() ;
virtual int peek_dim(int _4976) ;
virtual int peek_nd() ;
virtual void alloc_expr(int _4977,int _4978,int _4979,bool _4980) ;
virtual void move_expr(monty::rc_ptr< ::mosek::fusion::WorkStack > _4981) ;
virtual void peek_expr() ;
virtual void pop_expr() ;
virtual void ensure_sparsity() ;
virtual void clear() ;
virtual int allocf64(int _4996) ;
virtual int alloci64(int _4998) ;
virtual int alloci32(int _5000) ;
virtual void pushf64(double _5002) ;
virtual void pushi64(long long _5003) ;
virtual void pushi32(int _5004) ;
virtual void ensuref64(int _5005) ;
virtual void ensurei64(int _5008) ;
virtual void ensurei32(int _5011) ;
virtual int popf64(int _5014) ;
virtual int popi64(int _5015) ;
virtual int popi32(int _5016) ;
virtual void popf64(int _5017,std::shared_ptr< monty::ndarray< double,1 > > _5018,int _5019) ;
virtual void popi64(int _5020,std::shared_ptr< monty::ndarray< long long,1 > > _5021,int _5022) ;
virtual void popi32(int _5023,std::shared_ptr< monty::ndarray< int,1 > > _5024,int _5025) ;
virtual double popf64() ;
virtual long long popi64() ;
virtual int popi32() ;
virtual double peekf64() ;
virtual long long peeki64() ;
virtual int peeki32() ;
virtual double peekf64(int _5026) ;
virtual long long peeki64(int _5027) ;
virtual int peeki32(int _5028) ;
}; // struct WorkStack;

struct p_SymmetricExpr
{
SymmetricExpr * _pubthis;
static mosek::fusion::p_SymmetricExpr* _get_impl(mosek::fusion::SymmetricExpr * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_SymmetricExpr * _get_impl(mosek::fusion::SymmetricExpr::t _inst) { return _get_impl(_inst.get()); }
p_SymmetricExpr(SymmetricExpr * _pubthis);
virtual ~p_SymmetricExpr() { /* std::cout << "~p_SymmetricExpr" << std::endl;*/ };
std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Variable >,1 > > xs{};monty::rc_ptr< ::mosek::fusion::SymmetricMatrix > b{};std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::SymmetricMatrix >,1 > > Ms{};int n{};virtual void destroy();
static SymmetricExpr::t _new_SymmetricExpr(int _5029,std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::SymmetricMatrix >,1 > > _5030,std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Variable >,1 > > _5031,monty::rc_ptr< ::mosek::fusion::SymmetricMatrix > _5032);
void _initialize(int _5029,std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::SymmetricMatrix >,1 > > _5030,std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Variable >,1 > > _5031,monty::rc_ptr< ::mosek::fusion::SymmetricMatrix > _5032);
static  monty::rc_ptr< ::mosek::fusion::SymmetricExpr > add(monty::rc_ptr< ::mosek::fusion::SymmetricExpr > _5033,monty::rc_ptr< ::mosek::fusion::SymmetricMatrix > _5034);
static  monty::rc_ptr< ::mosek::fusion::SymmetricExpr > mul(monty::rc_ptr< ::mosek::fusion::SymmetricExpr > _5035,double _5036);
static  monty::rc_ptr< ::mosek::fusion::SymmetricExpr > add(monty::rc_ptr< ::mosek::fusion::SymmetricExpr > _5038,monty::rc_ptr< ::mosek::fusion::SymmetricExpr > _5039);
virtual /* override */ std::string toString() ;
}; // struct SymmetricExpr;

struct p_FlatExpr
{
FlatExpr * _pubthis;
static mosek::fusion::p_FlatExpr* _get_impl(mosek::fusion::FlatExpr * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_FlatExpr * _get_impl(mosek::fusion::FlatExpr::t _inst) { return _get_impl(_inst.get()); }
p_FlatExpr(FlatExpr * _pubthis);
virtual ~p_FlatExpr() { /* std::cout << "~p_FlatExpr" << std::endl;*/ };
std::shared_ptr< monty::ndarray< long long,1 > > inst{};std::shared_ptr< monty::ndarray< int,1 > > shape{};long long nnz{};std::shared_ptr< monty::ndarray< double,1 > > cof{};std::shared_ptr< monty::ndarray< long long,1 > > subj{};std::shared_ptr< monty::ndarray< long long,1 > > ptrb{};std::shared_ptr< monty::ndarray< double,1 > > bfix{};virtual void destroy();
static FlatExpr::t _new_FlatExpr(monty::rc_ptr< ::mosek::fusion::FlatExpr > _5712);
void _initialize(monty::rc_ptr< ::mosek::fusion::FlatExpr > _5712);
static FlatExpr::t _new_FlatExpr(std::shared_ptr< monty::ndarray< double,1 > > _5713,std::shared_ptr< monty::ndarray< long long,1 > > _5714,std::shared_ptr< monty::ndarray< long long,1 > > _5715,std::shared_ptr< monty::ndarray< double,1 > > _5716,std::shared_ptr< monty::ndarray< int,1 > > _5717,std::shared_ptr< monty::ndarray< long long,1 > > _5718);
void _initialize(std::shared_ptr< monty::ndarray< double,1 > > _5713,std::shared_ptr< monty::ndarray< long long,1 > > _5714,std::shared_ptr< monty::ndarray< long long,1 > > _5715,std::shared_ptr< monty::ndarray< double,1 > > _5716,std::shared_ptr< monty::ndarray< int,1 > > _5717,std::shared_ptr< monty::ndarray< long long,1 > > _5718);
virtual /* override */ std::string toString() ;
virtual int size() ;
}; // struct FlatExpr;

struct p_SymmetricMatrix
{
SymmetricMatrix * _pubthis;
static mosek::fusion::p_SymmetricMatrix* _get_impl(mosek::fusion::SymmetricMatrix * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_SymmetricMatrix * _get_impl(mosek::fusion::SymmetricMatrix::t _inst) { return _get_impl(_inst.get()); }
p_SymmetricMatrix(SymmetricMatrix * _pubthis);
virtual ~p_SymmetricMatrix() { /* std::cout << "~p_SymmetricMatrix" << std::endl;*/ };
int nnz{};double scale{};std::shared_ptr< monty::ndarray< double,1 > > vval{};std::shared_ptr< monty::ndarray< int,1 > > vsubj{};std::shared_ptr< monty::ndarray< int,1 > > vsubi{};std::shared_ptr< monty::ndarray< double,1 > > uval{};std::shared_ptr< monty::ndarray< int,1 > > usubj{};std::shared_ptr< monty::ndarray< int,1 > > usubi{};int d1{};int d0{};virtual void destroy();
static SymmetricMatrix::t _new_SymmetricMatrix(int _5720,int _5721,std::shared_ptr< monty::ndarray< int,1 > > _5722,std::shared_ptr< monty::ndarray< int,1 > > _5723,std::shared_ptr< monty::ndarray< double,1 > > _5724,std::shared_ptr< monty::ndarray< int,1 > > _5725,std::shared_ptr< monty::ndarray< int,1 > > _5726,std::shared_ptr< monty::ndarray< double,1 > > _5727,double _5728);
void _initialize(int _5720,int _5721,std::shared_ptr< monty::ndarray< int,1 > > _5722,std::shared_ptr< monty::ndarray< int,1 > > _5723,std::shared_ptr< monty::ndarray< double,1 > > _5724,std::shared_ptr< monty::ndarray< int,1 > > _5725,std::shared_ptr< monty::ndarray< int,1 > > _5726,std::shared_ptr< monty::ndarray< double,1 > > _5727,double _5728);
static  monty::rc_ptr< ::mosek::fusion::SymmetricMatrix > rankOne(int _5729,std::shared_ptr< monty::ndarray< int,1 > > _5730,std::shared_ptr< monty::ndarray< double,1 > > _5731);
static  monty::rc_ptr< ::mosek::fusion::SymmetricMatrix > rankOne(std::shared_ptr< monty::ndarray< double,1 > > _5739);
static  monty::rc_ptr< ::mosek::fusion::SymmetricMatrix > antiDiag(std::shared_ptr< monty::ndarray< double,1 > > _5747);
static  monty::rc_ptr< ::mosek::fusion::SymmetricMatrix > diag(std::shared_ptr< monty::ndarray< double,1 > > _5754);
virtual monty::rc_ptr< ::mosek::fusion::SymmetricMatrix > __mosek_2fusion_2SymmetricMatrix__add(monty::rc_ptr< ::mosek::fusion::SymmetricMatrix > _5760) ;
virtual monty::rc_ptr< ::mosek::fusion::SymmetricMatrix > __mosek_2fusion_2SymmetricMatrix__sub(monty::rc_ptr< ::mosek::fusion::SymmetricMatrix > _5780) ;
virtual monty::rc_ptr< ::mosek::fusion::SymmetricMatrix > __mosek_2fusion_2SymmetricMatrix__mul(double _5781) ;
virtual int getdim() ;
}; // struct SymmetricMatrix;

struct p_NDSparseArray
{
NDSparseArray * _pubthis;
static mosek::fusion::p_NDSparseArray* _get_impl(mosek::fusion::NDSparseArray * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_NDSparseArray * _get_impl(mosek::fusion::NDSparseArray::t _inst) { return _get_impl(_inst.get()); }
p_NDSparseArray(NDSparseArray * _pubthis);
virtual ~p_NDSparseArray() { /* std::cout << "~p_NDSparseArray" << std::endl;*/ };
std::shared_ptr< monty::ndarray< double,1 > > cof{};std::shared_ptr< monty::ndarray< long long,1 > > inst{};std::shared_ptr< monty::ndarray< int,1 > > dims{};long long size{};virtual void destroy();
static NDSparseArray::t _new_NDSparseArray(std::shared_ptr< monty::ndarray< int,1 > > _5782,std::shared_ptr< monty::ndarray< int,2 > > _5783,std::shared_ptr< monty::ndarray< double,1 > > _5784);
void _initialize(std::shared_ptr< monty::ndarray< int,1 > > _5782,std::shared_ptr< monty::ndarray< int,2 > > _5783,std::shared_ptr< monty::ndarray< double,1 > > _5784);
static NDSparseArray::t _new_NDSparseArray(std::shared_ptr< monty::ndarray< int,1 > > _5805,std::shared_ptr< monty::ndarray< long long,1 > > _5806,std::shared_ptr< monty::ndarray< double,1 > > _5807);
void _initialize(std::shared_ptr< monty::ndarray< int,1 > > _5805,std::shared_ptr< monty::ndarray< long long,1 > > _5806,std::shared_ptr< monty::ndarray< double,1 > > _5807);
static NDSparseArray::t _new_NDSparseArray(monty::rc_ptr< ::mosek::fusion::Matrix > _5822);
void _initialize(monty::rc_ptr< ::mosek::fusion::Matrix > _5822);
static  monty::rc_ptr< ::mosek::fusion::NDSparseArray > make(monty::rc_ptr< ::mosek::fusion::Matrix > _5830);
static  monty::rc_ptr< ::mosek::fusion::NDSparseArray > make(std::shared_ptr< monty::ndarray< int,1 > > _5831,std::shared_ptr< monty::ndarray< long long,1 > > _5832,std::shared_ptr< monty::ndarray< double,1 > > _5833);
static  monty::rc_ptr< ::mosek::fusion::NDSparseArray > make(std::shared_ptr< monty::ndarray< int,1 > > _5834,std::shared_ptr< monty::ndarray< int,2 > > _5835,std::shared_ptr< monty::ndarray< double,1 > > _5836);
}; // struct NDSparseArray;

struct p_Matrix
{
Matrix * _pubthis;
static mosek::fusion::p_Matrix* _get_impl(mosek::fusion::Matrix * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_Matrix * _get_impl(mosek::fusion::Matrix::t _inst) { return _get_impl(_inst.get()); }
p_Matrix(Matrix * _pubthis);
virtual ~p_Matrix() { /* std::cout << "~p_Matrix" << std::endl;*/ };
int dimj{};int dimi{};virtual void destroy();
static Matrix::t _new_Matrix(int _5906,int _5907);
void _initialize(int _5906,int _5907);
virtual /* override */ std::string toString() ;
virtual void switchDims() ;
static  monty::rc_ptr< ::mosek::fusion::Matrix > diag(int _5909,monty::rc_ptr< ::mosek::fusion::Matrix > _5910);
static  monty::rc_ptr< ::mosek::fusion::Matrix > diag(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Matrix >,1 > > _5912);
static  monty::rc_ptr< ::mosek::fusion::Matrix > antidiag(int _5930,double _5931,int _5932);
static  monty::rc_ptr< ::mosek::fusion::Matrix > antidiag(int _5933,double _5934);
static  monty::rc_ptr< ::mosek::fusion::Matrix > diag(int _5935,double _5936,int _5937);
static  monty::rc_ptr< ::mosek::fusion::Matrix > diag(int _5938,double _5939);
static  monty::rc_ptr< ::mosek::fusion::Matrix > antidiag(std::shared_ptr< monty::ndarray< double,1 > > _5940,int _5941);
static  monty::rc_ptr< ::mosek::fusion::Matrix > antidiag(std::shared_ptr< monty::ndarray< double,1 > > _5951);
static  monty::rc_ptr< ::mosek::fusion::Matrix > diag(std::shared_ptr< monty::ndarray< double,1 > > _5952,int _5953);
static  monty::rc_ptr< ::mosek::fusion::Matrix > diag(std::shared_ptr< monty::ndarray< double,1 > > _5961);
static  monty::rc_ptr< ::mosek::fusion::Matrix > ones(int _5962,int _5963);
static  monty::rc_ptr< ::mosek::fusion::Matrix > eye(int _5964);
static  monty::rc_ptr< ::mosek::fusion::Matrix > dense(monty::rc_ptr< ::mosek::fusion::Matrix > _5966);
static  monty::rc_ptr< ::mosek::fusion::Matrix > dense(int _5967,int _5968,double _5969);
static  monty::rc_ptr< ::mosek::fusion::Matrix > dense(int _5970,int _5971,std::shared_ptr< monty::ndarray< double,1 > > _5972);
static  monty::rc_ptr< ::mosek::fusion::Matrix > dense(std::shared_ptr< monty::ndarray< double,2 > > _5973);
static  monty::rc_ptr< ::mosek::fusion::Matrix > sparse(monty::rc_ptr< ::mosek::fusion::Matrix > _5974);
static  monty::rc_ptr< ::mosek::fusion::Matrix > sparse(std::shared_ptr< monty::ndarray< std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Matrix >,1 > >,1 > > _5978);
static  monty::rc_ptr< ::mosek::fusion::Matrix > sparse(std::shared_ptr< monty::ndarray< double,2 > > _6009);
static  monty::rc_ptr< ::mosek::fusion::Matrix > sparse(int _6019,int _6020);
static  monty::rc_ptr< ::mosek::fusion::Matrix > sparse(int _6021,int _6022,std::shared_ptr< monty::ndarray< int,1 > > _6023,std::shared_ptr< monty::ndarray< int,1 > > _6024,double _6025);
static  monty::rc_ptr< ::mosek::fusion::Matrix > sparse(std::shared_ptr< monty::ndarray< int,1 > > _6027,std::shared_ptr< monty::ndarray< int,1 > > _6028,double _6029);
static  monty::rc_ptr< ::mosek::fusion::Matrix > sparse(std::shared_ptr< monty::ndarray< int,1 > > _6034,std::shared_ptr< monty::ndarray< int,1 > > _6035,std::shared_ptr< monty::ndarray< double,1 > > _6036);
static  monty::rc_ptr< ::mosek::fusion::Matrix > sparse(int _6041,int _6042,std::shared_ptr< monty::ndarray< int,1 > > _6043,std::shared_ptr< monty::ndarray< int,1 > > _6044,std::shared_ptr< monty::ndarray< double,1 > > _6045);
virtual double get(int _6050,int _6051) { throw monty::AbstractClassError("Call to abstract method"); }
virtual bool isSparse() { throw monty::AbstractClassError("Call to abstract method"); }
virtual std::shared_ptr< monty::ndarray< double,1 > > getDataAsArray() { throw monty::AbstractClassError("Call to abstract method"); }
virtual void getDataAsTriplets(std::shared_ptr< monty::ndarray< int,1 > > _6052,std::shared_ptr< monty::ndarray< int,1 > > _6053,std::shared_ptr< monty::ndarray< double,1 > > _6054) { throw monty::AbstractClassError("Call to abstract method"); }
virtual monty::rc_ptr< ::mosek::fusion::Matrix > __mosek_2fusion_2Matrix__transpose() { throw monty::AbstractClassError("Call to abstract method"); }
virtual long long numNonzeros() { throw monty::AbstractClassError("Call to abstract method"); }
virtual int numColumns() ;
virtual int numRows() ;
}; // struct Matrix;

struct p_DenseMatrix : public ::mosek::fusion::p_Matrix
{
DenseMatrix * _pubthis;
static mosek::fusion::p_DenseMatrix* _get_impl(mosek::fusion::DenseMatrix * _inst){ return static_cast< mosek::fusion::p_DenseMatrix* >(mosek::fusion::p_Matrix::_get_impl(_inst)); }
static mosek::fusion::p_DenseMatrix * _get_impl(mosek::fusion::DenseMatrix::t _inst) { return _get_impl(_inst.get()); }
p_DenseMatrix(DenseMatrix * _pubthis);
virtual ~p_DenseMatrix() { /* std::cout << "~p_DenseMatrix" << std::endl;*/ };
long long nnz{};std::shared_ptr< monty::ndarray< double,1 > > data{};virtual void destroy();
static DenseMatrix::t _new_DenseMatrix(int _5837,int _5838,std::shared_ptr< monty::ndarray< double,1 > > _5839);
void _initialize(int _5837,int _5838,std::shared_ptr< monty::ndarray< double,1 > > _5839);
static DenseMatrix::t _new_DenseMatrix(monty::rc_ptr< ::mosek::fusion::Matrix > _5840);
void _initialize(monty::rc_ptr< ::mosek::fusion::Matrix > _5840);
static DenseMatrix::t _new_DenseMatrix(std::shared_ptr< monty::ndarray< double,2 > > _5845);
void _initialize(std::shared_ptr< monty::ndarray< double,2 > > _5845);
static DenseMatrix::t _new_DenseMatrix(int _5848,int _5849,double _5850);
void _initialize(int _5848,int _5849,double _5850);
virtual /* override */ std::string toString() ;
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Matrix > __mosek_2fusion_2DenseMatrix__transpose() ;
virtual monty::rc_ptr< ::mosek::fusion::Matrix > __mosek_2fusion_2Matrix__transpose() { return __mosek_2fusion_2DenseMatrix__transpose(); }
virtual /* override */ bool isSparse() ;
virtual /* override */ std::shared_ptr< monty::ndarray< double,1 > > getDataAsArray() ;
virtual /* override */ void getDataAsTriplets(std::shared_ptr< monty::ndarray< int,1 > > _5863,std::shared_ptr< monty::ndarray< int,1 > > _5864,std::shared_ptr< monty::ndarray< double,1 > > _5865) ;
virtual /* override */ double get(int _5869,int _5870) ;
virtual /* override */ long long numNonzeros() ;
}; // struct DenseMatrix;

struct p_SparseMatrix : public ::mosek::fusion::p_Matrix
{
SparseMatrix * _pubthis;
static mosek::fusion::p_SparseMatrix* _get_impl(mosek::fusion::SparseMatrix * _inst){ return static_cast< mosek::fusion::p_SparseMatrix* >(mosek::fusion::p_Matrix::_get_impl(_inst)); }
static mosek::fusion::p_SparseMatrix * _get_impl(mosek::fusion::SparseMatrix::t _inst) { return _get_impl(_inst.get()); }
p_SparseMatrix(SparseMatrix * _pubthis);
virtual ~p_SparseMatrix() { /* std::cout << "~p_SparseMatrix" << std::endl;*/ };
long long nnz{};std::shared_ptr< monty::ndarray< double,1 > > val{};std::shared_ptr< monty::ndarray< int,1 > > subj{};std::shared_ptr< monty::ndarray< int,1 > > subi{};virtual void destroy();
static SparseMatrix::t _new_SparseMatrix(int _5871,int _5872,std::shared_ptr< monty::ndarray< int,1 > > _5873,std::shared_ptr< monty::ndarray< int,1 > > _5874,std::shared_ptr< monty::ndarray< double,1 > > _5875,long long _5876);
void _initialize(int _5871,int _5872,std::shared_ptr< monty::ndarray< int,1 > > _5873,std::shared_ptr< monty::ndarray< int,1 > > _5874,std::shared_ptr< monty::ndarray< double,1 > > _5875,long long _5876);
static SparseMatrix::t _new_SparseMatrix(int _5882,int _5883,std::shared_ptr< monty::ndarray< int,1 > > _5884,std::shared_ptr< monty::ndarray< int,1 > > _5885,std::shared_ptr< monty::ndarray< double,1 > > _5886);
void _initialize(int _5882,int _5883,std::shared_ptr< monty::ndarray< int,1 > > _5884,std::shared_ptr< monty::ndarray< int,1 > > _5885,std::shared_ptr< monty::ndarray< double,1 > > _5886);
virtual std::shared_ptr< monty::ndarray< long long,1 > > formPtrb() ;
virtual /* override */ std::string toString() ;
virtual /* override */ long long numNonzeros() ;
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Matrix > __mosek_2fusion_2SparseMatrix__transpose() ;
virtual monty::rc_ptr< ::mosek::fusion::Matrix > __mosek_2fusion_2Matrix__transpose() { return __mosek_2fusion_2SparseMatrix__transpose(); }
virtual /* override */ bool isSparse() ;
virtual /* override */ std::shared_ptr< monty::ndarray< double,1 > > getDataAsArray() ;
virtual /* override */ void getDataAsTriplets(std::shared_ptr< monty::ndarray< int,1 > > _5898,std::shared_ptr< monty::ndarray< int,1 > > _5899,std::shared_ptr< monty::ndarray< double,1 > > _5900) ;
virtual /* override */ double get(int _5901,int _5902) ;
}; // struct SparseMatrix;

struct p_LinkedBlocks
{
LinkedBlocks * _pubthis;
static mosek::fusion::p_LinkedBlocks* _get_impl(mosek::fusion::LinkedBlocks * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_LinkedBlocks * _get_impl(mosek::fusion::LinkedBlocks::t _inst) { return _get_impl(_inst.get()); }
p_LinkedBlocks(LinkedBlocks * _pubthis);
virtual ~p_LinkedBlocks() { /* std::cout << "~p_LinkedBlocks" << std::endl;*/ };
std::shared_ptr< monty::ndarray< int,1 > > bfirst{};std::shared_ptr< monty::ndarray< int,1 > > bsize{};monty::rc_ptr< ::mosek::fusion::LinkedInts > blocks{};monty::rc_ptr< ::mosek::fusion::LinkedInts > ints{};virtual void destroy();
static LinkedBlocks::t _new_LinkedBlocks();
void _initialize();
static LinkedBlocks::t _new_LinkedBlocks(int _6078);
void _initialize(int _6078);
static LinkedBlocks::t _new_LinkedBlocks(monty::rc_ptr< ::mosek::fusion::LinkedBlocks > _6079);
void _initialize(monty::rc_ptr< ::mosek::fusion::LinkedBlocks > _6079);
virtual void free(int _6080) ;
virtual int alloc(int _6082) ;
virtual int maxidx(int _6087) ;
virtual void get(int _6088,std::shared_ptr< monty::ndarray< int,1 > > _6089,int _6090) ;
virtual int numblocks() ;
virtual int blocksize(int _6091) ;
virtual int capacity() ;
virtual bool validate() ;
}; // struct LinkedBlocks;

struct p_LinkedInts
{
LinkedInts * _pubthis;
static mosek::fusion::p_LinkedInts* _get_impl(mosek::fusion::LinkedInts * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_LinkedInts * _get_impl(mosek::fusion::LinkedInts::t _inst) { return _get_impl(_inst.get()); }
p_LinkedInts(LinkedInts * _pubthis);
virtual ~p_LinkedInts() { /* std::cout << "~p_LinkedInts" << std::endl;*/ };
int nfree{};int last_free{};int first_free{};int first_used{};std::shared_ptr< monty::ndarray< int,1 > > prev{};std::shared_ptr< monty::ndarray< int,1 > > next{};virtual void destroy();
static LinkedInts::t _new_LinkedInts(int _6092);
void _initialize(int _6092);
static LinkedInts::t _new_LinkedInts();
void _initialize();
static LinkedInts::t _new_LinkedInts(monty::rc_ptr< ::mosek::fusion::LinkedInts > _6095);
void _initialize(monty::rc_ptr< ::mosek::fusion::LinkedInts > _6095);
virtual void free(int _6096,int _6097) ;
virtual int alloc() ;
virtual int alloc(int _6103) ;
virtual void alloc(int _6104,std::shared_ptr< monty::ndarray< int,1 > > _6105,int _6106) ;
virtual void get(int _6109,int _6110,std::shared_ptr< monty::ndarray< int,1 > > _6111,int _6112) ;
virtual int maxidx(int _6115,int _6116) ;
virtual int allocblock(int _6120) ;
virtual void recap(int _6126) ;
virtual int capacity() ;
virtual bool validate() ;
}; // struct LinkedInts;

struct p_Parameters
{
Parameters * _pubthis;
static mosek::fusion::p_Parameters* _get_impl(mosek::fusion::Parameters * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_Parameters * _get_impl(mosek::fusion::Parameters::t _inst) { return _get_impl(_inst.get()); }
p_Parameters(Parameters * _pubthis);
virtual ~p_Parameters() { /* std::cout << "~p_Parameters" << std::endl;*/ };
virtual void destroy();
static  void setParameter(monty::rc_ptr< ::mosek::fusion::Model > _6135,const std::string &  _6136,double _6137);
static  void setParameter(monty::rc_ptr< ::mosek::fusion::Model > _6231,const std::string &  _6232,int _6233);
static  void setParameter(monty::rc_ptr< ::mosek::fusion::Model > _6327,const std::string &  _6328,const std::string &  _6329);
static  int string_to_variabletype_value(const std::string &  _6567);
static  int string_to_value_value(const std::string &  _6568);
static  int string_to_streamtype_value(const std::string &  _6569);
static  int string_to_startpointtype_value(const std::string &  _6570);
static  int string_to_stakey_value(const std::string &  _6571);
static  int string_to_sparam_value(const std::string &  _6572);
static  int string_to_solveform_value(const std::string &  _6573);
static  int string_to_soltype_value(const std::string &  _6574);
static  int string_to_solsta_value(const std::string &  _6575);
static  int string_to_solitem_value(const std::string &  _6576);
static  int string_to_simseltype_value(const std::string &  _6577);
static  int string_to_sensitivitytype_value(const std::string &  _6578);
static  int string_to_scalingmethod_value(const std::string &  _6579);
static  int string_to_scalingtype_value(const std::string &  _6580);
static  int string_to_rescodetype_value(const std::string &  _6581);
static  int string_to_rescode_value(const std::string &  _6582);
static  int string_to_xmlwriteroutputtype_value(const std::string &  _6583);
static  int string_to_prosta_value(const std::string &  _6584);
static  int string_to_problemtype_value(const std::string &  _6585);
static  int string_to_problemitem_value(const std::string &  _6586);
static  int string_to_parametertype_value(const std::string &  _6587);
static  int string_to_presolvemode_value(const std::string &  _6588);
static  int string_to_orderingtype_value(const std::string &  _6589);
static  int string_to_optimizertype_value(const std::string &  _6590);
static  int string_to_onoffkey_value(const std::string &  _6591);
static  int string_to_objsense_value(const std::string &  _6592);
static  int string_to_mpsformat_value(const std::string &  _6593);
static  int string_to_mionodeseltype_value(const std::string &  _6594);
static  int string_to_miomode_value(const std::string &  _6595);
static  int string_to_miocontsoltype_value(const std::string &  _6596);
static  int string_to_branchdir_value(const std::string &  _6597);
static  int string_to_iparam_value(const std::string &  _6598);
static  int string_to_iomode_value(const std::string &  _6599);
static  int string_to_internal_iinf_value(const std::string &  _6600);
static  int string_to_internal_dinf_value(const std::string &  _6601);
static  int string_to_inftype_value(const std::string &  _6602);
static  int string_to_iinfitem_value(const std::string &  _6603);
static  int string_to_internal_liinf_value(const std::string &  _6604);
static  int string_to_liinfitem_value(const std::string &  _6605);
static  int string_to_dparam_value(const std::string &  _6606);
static  int string_to_feature_value(const std::string &  _6607);
static  int string_to_dinfitem_value(const std::string &  _6608);
static  int string_to_dataformat_value(const std::string &  _6609);
static  int string_to_symmattype_value(const std::string &  _6610);
static  int string_to_scopr_value(const std::string &  _6611);
static  int string_to_nametype_value(const std::string &  _6612);
static  int string_to_conetype_value(const std::string &  _6613);
static  int string_to_compresstype_value(const std::string &  _6614);
static  int string_to_checkconvexitytype_value(const std::string &  _6615);
static  int string_to_callbackcode_value(const std::string &  _6616);
static  int string_to_purify_value(const std::string &  _6617);
static  int string_to_intpnthotstart_value(const std::string &  _6618);
static  int string_to_simhotstart_value(const std::string &  _6619);
static  int string_to_simdupvec_value(const std::string &  _6620);
static  int string_to_simreform_value(const std::string &  _6621);
static  int string_to_uplo_value(const std::string &  _6622);
static  int string_to_transpose_value(const std::string &  _6623);
static  int string_to_simdegen_value(const std::string &  _6624);
static  int string_to_mark_value(const std::string &  _6625);
static  int string_to_boundkey_value(const std::string &  _6626);
static  int string_to_basindtype_value(const std::string &  _6627);
static  int string_to_language_value(const std::string &  _6628);
}; // struct Parameters;

}
}
namespace mosek
{
namespace fusion
{
namespace Utils
{
// mosek.fusion.Utils.IntMap from file 'src\fusion\cxx\IntMap_p.h'
struct p_IntMap 
{
  IntMap * _pubself;

  static p_IntMap * _get_impl(IntMap * _inst) { return _inst->_impl.get(); }

  p_IntMap(IntMap * _pubself) : _pubself(_pubself) {}

  static IntMap::t _new_IntMap() { return new IntMap(); }

  ::std::unordered_map<long long,int> m;

  bool hasItem (long long key) { return m.find(key) != m.end(); }
  int  getItem (long long key) { return m.find(key)->second; } // will probably throw something or crash of no such key
  void setItem (long long key, int val) { m[key] = val; }

  std::shared_ptr<monty::ndarray<long long,1>> keys()
  { 
    size_t size = m.size();
    auto res = std::shared_ptr<monty::ndarray<long long,1>>(new monty::ndarray<long long,1>(monty::shape((int)size)));

    ptrdiff_t i = 0;
    for (auto& it : m)
	    (*res)[i++] = it.first;

    return res;    
  }

  std::shared_ptr<monty::ndarray<int,1>> values()
  {
    size_t size = m.size();
    auto res = std::shared_ptr<monty::ndarray<int,1>>(new monty::ndarray<int,1>(monty::shape((int)size)));

    ptrdiff_t i = 0;
    for (auto& it : m)
	    (*res)[i++] = it.second;

    return res;
  }

  IntMap::t clone();
  IntMap::t __mosek_2fusion_2Utils_2IntMap__clone();
};



struct p_StringIntMap
{
  StringIntMap * _pubself;

  static p_StringIntMap * _get_impl(StringIntMap * _inst) { return _inst->_impl.get(); }

  p_StringIntMap(StringIntMap * _pubself) : _pubself(_pubself) {}

  static StringIntMap::t _new_StringIntMap() { return new StringIntMap(); }

  ::std::unordered_map<std::string,int> m;

  bool hasItem (const std::string & key) { return m.find(key) != m.end(); }
  int  getItem (const std::string & key) { return m.find(key)->second; } // will probably throw something or crash of no such key
  void setItem (const std::string & key, int val) { m[key] = val; }

  std::shared_ptr<monty::ndarray<std::string,1>> keys()
  {
    size_t size = m.size();
    auto res = std::shared_ptr<monty::ndarray<std::string,1>>(new monty::ndarray<std::string,1>(monty::shape((int)size)));

    ptrdiff_t i = 0;
    for (auto& it : m)
	    (*res)[i++] = it.first;

    return res;
  }

  std::shared_ptr<monty::ndarray<int,1>> values()
  {
    size_t size = m.size();
    auto res = std::shared_ptr<monty::ndarray<int,1>>(new monty::ndarray<int,1>(monty::shape((int)size)));

    ptrdiff_t i = 0;
    for (auto& it : m)
	    (*res)[i++] = it.second;

    return res;
  }

  StringIntMap::t clone();
  StringIntMap::t __mosek_2fusion_2Utils_2StringIntMap__clone();
};
// End of file 'src\fusion\cxx\IntMap_p.h'
// mosek.fusion.Utils.StringBuffer from file 'src\fusion\cxx\StringBuffer_p.h'
// namespace mosek::fusion::Utils
struct p_StringBuffer
{
  StringBuffer * _pubthis; 
  std::stringstream ss;

  p_StringBuffer(StringBuffer * _pubthis) : _pubthis(_pubthis) {}

  static p_StringBuffer * _get_impl(StringBuffer::t ptr) { return ptr->_impl.get(); }
  static p_StringBuffer * _get_impl(StringBuffer * ptr) { return ptr->_impl.get(); }

  static StringBuffer::t _new_StringBuffer() { return new StringBuffer(); }

  StringBuffer::t clear ();
  
  StringBuffer::t a (const monty::ndarray<std::string,1> & val);
  StringBuffer::t a (const monty::ndarray<int,1> & val);
  StringBuffer::t a (const monty::ndarray<long long,1> & val);
  StringBuffer::t a (const monty::ndarray<double,1> & val);
  

  StringBuffer::t a (const int & val);
  StringBuffer::t a (const long long & val);
  StringBuffer::t a (const double & val);
  StringBuffer::t a (const bool & val);
  StringBuffer::t a (const std::string & val);

  StringBuffer::t lf ();
  StringBuffer::t __mosek_2fusion_2Utils_2StringBuffer__clear ();

  StringBuffer::t __mosek_2fusion_2Utils_2StringBuffer__a (const monty::ndarray<std::string,1> & val);
  StringBuffer::t __mosek_2fusion_2Utils_2StringBuffer__a (const monty::ndarray<int,1> & val);
  StringBuffer::t __mosek_2fusion_2Utils_2StringBuffer__a (const monty::ndarray<long long,1> & val);
  StringBuffer::t __mosek_2fusion_2Utils_2StringBuffer__a (const monty::ndarray<double,1> & val);

  StringBuffer::t __mosek_2fusion_2Utils_2StringBuffer__a (const int & val);
  StringBuffer::t __mosek_2fusion_2Utils_2StringBuffer__a (const long long & val);
  StringBuffer::t __mosek_2fusion_2Utils_2StringBuffer__a (const double & val);
  StringBuffer::t __mosek_2fusion_2Utils_2StringBuffer__a (const bool & val);
  StringBuffer::t __mosek_2fusion_2Utils_2StringBuffer__a (const std::string & val);

  StringBuffer::t __mosek_2fusion_2Utils_2StringBuffer__lf ();

  std::string toString () const;
};
// End of file 'src\fusion\cxx\StringBuffer_p.h'
}
}
}
#endif
