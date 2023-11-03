#include "LineQuadrature.hpp"

#include <cassert>
#include <cmath>
#include <vector>

namespace wmtk {

std::vector<double> quadrature_points(const int order)
{
    switch (order) {
    case 1: return {0};
    case 2:
        return {
            -0.5773502691896257645091487805019574556476017512701268760186023264839776723029333456937153955857495252252087138051355676766566483649996508262705518373647912161760310773007685273559916067003615583077550051041144223011076288835574182229739459904090157105534559538626730166621791266197964892168,
            0.5773502691896257645091487805019574556476017512701268760186023264839776723029333456937153955857495252252087138051355676766566483649996508262705518373647912161760310773007685273559916067003615583077550051041144223011076288835574182229739459904090157105534559538626730166621791266197964892168,
        };

    case 3:
        return {
            0,
            -0.774596669241483377035853079956479922166584341058318165317514753222696618387395806703857475371734703583260441372189929402637908087832729923135978349224240702213750958202698716256783906245777858513169283405612501838634682531972963691092925710263188052523534528101729260090115562126394576188,
            0.774596669241483377035853079956479922166584341058318165317514753222696618387395806703857475371734703583260441372189929402637908087832729923135978349224240702213750958202698716256783906245777858513169283405612501838634682531972963691092925710263188052523534528101729260090115562126394576188,
        };

    case 4:
        return {
            -0.3399810435848562648026657591032446872005758697709143525929539768210200304632370344778752804355548115489602395207464932135845003241712491992776363684338328221538611182352836311104158340621521124125023821932864240034767086752629560943410821534146791671405442668508151756169732898924953195536,
            0.3399810435848562648026657591032446872005758697709143525929539768210200304632370344778752804355548115489602395207464932135845003241712491992776363684338328221538611182352836311104158340621521124125023821932864240034767086752629560943410821534146791671405442668508151756169732898924953195536,
            -0.8611363115940525752239464888928095050957253796297176376157219209065294714950488657041623398844793052105769209319781763249637438391157919764084938458618855762872931327441369944290122598469710261906458681564745219362114916066097678053187180580268539141223471780870198639372247416951073770551,
            0.8611363115940525752239464888928095050957253796297176376157219209065294714950488657041623398844793052105769209319781763249637438391157919764084938458618855762872931327441369944290122598469710261906458681564745219362114916066097678053187180580268539141223471780870198639372247416951073770551,
        };

    case 5:
        return {
            0,
            -0.5384693101056830910363144207002088049672866069055599562022316270594711853677552910358036672505709315713670572321043495510816912158744046420683486075627481533978123828583369317846132387526796166796502053799563629878671716361660767584852200097418079241406256057571019602720019270523093750336,
            0.5384693101056830910363144207002088049672866069055599562022316270594711853677552910358036672505709315713670572321043495510816912158744046420683486075627481533978123828583369317846132387526796166796502053799563629878671716361660767584852200097418079241406256057571019602720019270523093750336,
            -0.9061798459386639927976268782993929651256519107625308628737622865437707949166868469411429895535422619115836248167051160932020660084349721915374869570125418659061700540273012086530604091207821562942704193786707298217315368769002376029537907738935528847397895557648103916797868140600953498906,
            0.9061798459386639927976268782993929651256519107625308628737622865437707949166868469411429895535422619115836248167051160932020660084349721915374869570125418659061700540273012086530604091207821562942704193786707298217315368769002376029537907738935528847397895557648103916797868140600953498906,
        };


    default: {
        assert(false);
        return std::vector<double>();
    }
    }
} // namespace

std::vector<double> quadrature_weights(const int order)
{
    switch (order) {
    case 1: return {2};
    case 2:
        return {
            1.000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000,
            1.000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000,
        };

    case 3:
        return {
            0.8888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888889,
            0.5555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555556,
            0.5555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555556,
        };

    case 4:
        return {
            0.6521451548625461426269360507780005927646513041661064595074706804812481325340896482780162322677418404902018960952364978455755577496740182191429757016783303751407135229556360801973666260481564013273531860737119707353160256000107787211587578617532049337456560923057986412084590467808124974086,
            0.6521451548625461426269360507780005927646513041661064595074706804812481325340896482780162322677418404902018960952364978455755577496740182191429757016783303751407135229556360801973666260481564013273531860737119707353160256000107787211587578617532049337456560923057986412084590467808124974086,
            0.3478548451374538573730639492219994072353486958338935404925293195187518674659103517219837677322581595097981039047635021544244422503259817808570242983216696248592864770443639198026333739518435986726468139262880292646839743999892212788412421382467950662543439076942013587915409532191875025701,
            0.3478548451374538573730639492219994072353486958338935404925293195187518674659103517219837677322581595097981039047635021544244422503259817808570242983216696248592864770443639198026333739518435986726468139262880292646839743999892212788412421382467950662543439076942013587915409532191875025701,
        };

    case 5:
        return {
            0.5688888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888889,
            0.4786286704993664680412915148356381929122955533431415399727276673338382671525124569755621250616041107794464209474122299742927901670531874220236019762755381069981020199559708433435017355341690324695622104863536721598859262913644828482640505637133513606531929893286127565185389732581634388813,
            0.4786286704993664680412915148356381929122955533431415399727276673338382671525124569755621250616041107794464209474122299742927901670531874220236019762755381069981020199559708433435017355341690324695622104863536721598859262913644828482640505637133513606531929893286127565185389732581634388813,
            0.2369268850561890875142640407199173626432600022124140155828278882217172884030430985799934304939514447761091346081433255812627653885023681335319535792800174485574535355995847122120538200213865230859933450692018833956696292641910727072915049918422041949023625662269427990370165822973921166843,
            0.2369268850561890875142640407199173626432600022124140155828278882217172884030430985799934304939514447761091346081433255812627653885023681335319535792800174485574535355995847122120538200213865230859933450692018833956696292641910727072915049918422041949023625662269427990370165822973921166843,
        };

    default: {
        assert(false);
        return std::vector<double>();
    }
    }
} // namespace

void LineQuadrature::get_quadrature(const int order)
{
    std::vector<double> xi = quadrature_points(order);
    std::vector<double> wi = quadrature_weights(order);

    assert((xi.size() == wi.size()));

    std::sort(wi.begin(), wi.end(), [xi](int i1, int i2) { return xi[i1] < xi[i2]; });

    std::sort(xi.begin(), xi.end(), [xi](int i1, int i2) { return xi[i1] < xi[i2]; });

    points = Eigen::Map<Eigen::MatrixXd>(&xi[0], xi.size(), 1);
    weights = Eigen::Map<Eigen::MatrixXd>(&wi[0], wi.size(), 1);

    weights *= 0.5;
    points *= 0.5;
    points += Eigen::MatrixXd::Ones(points.rows(), points.cols()) * 0.5;

    assert(fabs(weights.sum() - 1) < 1e-14);
    assert(points.minCoeff() >= 0 && points.maxCoeff() <= 1);

    assert((points.size() == weights.size()));
    weights /= weights.sum();
}
} // namespace wmtk