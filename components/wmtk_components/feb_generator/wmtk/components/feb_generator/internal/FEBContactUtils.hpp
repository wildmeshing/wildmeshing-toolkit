#include <fstream>

namespace wmtk::components::internal {

void write_contact_template(
    std::ofstream& stream,
    std::string type,
    std::string contact_name,
    std::string surface_pair_name)
{
    if (type == "contact potential") {
        stream << "\t<contact name=\"" << contact_name << "\" surface_pair=\"" << surface_pair_name
               << "\" type=\"contact potential\">\n"
               << "\t\t<kc>0</kc>\n"
               << "\t\t<p>4</p>\n"
               << "\t\t<R_in>1</R_in>\n"
               << "\t\t<R_out>2</R_out>\n"
               << "\t\t<R0_min>0</R0_min>\n"
               << "\t\t<w_tol>0</w_tol>\n"
               << "\t</contact>\n";
    } else if (type == "periodic boundary") {
        stream << "\t<contact name=\"" << contact_name << "\" surface_pair=\"" << surface_pair_name
               << "\" type=\"periodic boundary\">\n"
               << "\t\t<laugon>PENALTY</laugon>\n"
               << "\t\t<tolerance>0</tolerance>\n"
               << "\t\t<penalty>0</penalty>\n"
               << "\t\t<two_pass>0</two_pass>\n"
               << "\t\t<offset>0,0,0</offset>\n"
               << "\t\t<minaug>0</minaug>\n"
               << "\t</contact>\n";
    } else if (type == "periodic boundary1O") {
        stream << "\t<contact name=\"" << contact_name << "\" surface_pair=\"" << surface_pair_name
               << "\" type=\"periodic boundary1O\">\n"
               << "\t\t<laugon>PENALTY</laugon>\n"
               << "\t\t<tolerance>0</tolerance>\n"
               << "\t\t<penalty>0</penalty>\n"
               << "\t\t<two_pass>0</two_pass>\n"
               << "\t\t<offset>0,0,0</offset>\n"
               << "\t\t<minaug>0</minaug>\n"
               << "\t</contact>\n";
    } else if (type == "periodic boundary2O") {
        stream << "\t<contact name=\"" << contact_name << "\" surface_pair=\"" << surface_pair_name
               << "\" type=\"periodic boundary2O\">\n"
               << "\t\t<laugon>PENALTY</laugon>\n"
               << "\t\t<tolerance>0</tolerance>\n"
               << "\t\t<penalty>0</penalty>\n"
               << "\t\t<two_pass>0</two_pass>\n"
               << "\t\t<offset>0,0,0</offset>\n"
               << "\t\t<minaug>0</minaug>\n"
               << "\t</contact>\n";
    } else if (type == "sliding-elastic") {
        stream << "\t<contact name=\"" << contact_name << "\" surface_pair=\"" << surface_pair_name
               << "\" type=\"sliding-elastic\">\n"
               << "\t\t<laugon>PENALTY</laugon>\n"
               << "\t\t<tolerance>0.1</tolerance>\n"
               << "\t\t<gaptol>0</gaptol>\n"
               << "\t\t<penalty>1</penalty>\n"
               << "\t\t<auto_penalty>0</auto_penalty>\n"
               << "\t\t<update_penalty>0</update_penalty>\n"
               << "\t\t<two_pass>0</two_pass>\n"
               << "\t\t<knmult>0</knmult>\n"
               << "\t\t<search_tol>0.01</search_tol>\n"
               << "\t\t<symmetric_stiffness>1</symmetric_stiffness>\n"
               << "\t\t<search_radius>1</search_radius>\n"
               << "\t\t<seg_up>0</seg_up>\n"
               << "\t\t<tension>0</tension>\n"
               << "\t\t<minaug>0</minaug>\n"
               << "\t\t<maxaug>10</maxaug>\n"
               << "\t\t<node_reloc>0</node_reloc>\n"
               << "\t\t<fric_coeff>0</fric_coeff>\n"
               << "\t\t<smooth_aug>0</smooth_aug>\n"
               << "\t\t<flip_primary>0</flip_primary>\n"
               << "\t\t<flip_secondary>0</flip_secondary>\n"
               << "\t\t<shell_bottom_primary>0</shell_bottom_primary>\n"
               << "\t\t<shell_bottom_secondary>0</shell_bottom_secondary>\n"
               << "\t\t<offset>0</offset>\n"
               << "\t</contact>\n";
    } else if (type == "sliding-facet-on-facet") {
        stream << "\t<contact name=\"" << contact_name << "\" surface_pair=\"" << surface_pair_name
               << "\" type=\"sliding-facet-on-facet\">\n"
               << "\t\t<laugon>PENALTY</laugon>\n"
               << "\t\t<penalty>1</penalty>\n"
               << "\t\t<auto_penalty>0</auto_penalty>\n"
               << "\t\t<update_penalty>0</update_penalty>\n"
               << "\t\t<tolerance>0.01</tolerance>\n"
               << "\t\t<two_pass>0</two_pass>\n"
               << "\t\t<gaptol>0</gaptol>\n"
               << "\t\t<minaug>0</minaug>\n"
               << "\t\t<maxaug>10</maxaug>\n"
               << "\t\t<smooth_aug>0</smooth_aug>\n"
               << "\t\t<search_tol>0.01</search_tol>\n"
               << "\t\t<search_radius>0</search_radius>\n"
               << "\t\t<seg_up>0</seg_up>\n"
               << "\t\t<node_reloc>0</node_reloc>\n"
               << "\t\t<knmult>1</knmult>\n"
               << "\t</contact>\n";
    } else if (type == "sliding-node-on-facet") {
        stream << "\t<contact name=\"" << contact_name << "\" surface_pair=\"" << surface_pair_name
               << "\" type=\"sliding-node-on-facet\">\n"
               << "\t\t<laugon>PENALTY</laugon>\n"
               << "\t\t<tolerance>6.605079e-223</tolerance>\n"
               << "\t\t<penalty>3.055674e-312</penalty>\n"
               << "\t\t<auto_penalty>0</auto_penalty>\n"
               << "\t\t<two_pass>0</two_pass>\n"
               << "\t\t<gaptol>0</gaptol>\n"
               << "\t\t<fric_coeff>0</fric_coeff>\n"
               << "\t\t<fric_penalty>0</fric_penalty>\n"
               << "\t\t<minaug>0</minaug>\n"
               << "\t\t<maxaug>10</maxaug>\n"
               << "\t\t<search_tol>0.01</search_tol>\n"
               << "\t\t<ktmult>0</ktmult>\n"
               << "\t\t<knmult>1</knmult>\n"
               << "\t\t<node_reloc>0</node_reloc>\n"
               << "\t\t<seg_up>0</seg_up>\n"
               << "\t\t<search_radius>0</search_radius>\n"
               << "\t\t<update_penalty>0</update_penalty>\n"
               << "\t</contact>\n";
    } else if (type == "sticky") {
        stream << "\t<contact name=\"" << contact_name << "\" surface_pair=\"" << surface_pair_name
               << "\" type=\"sticky\">\n"
               << "\t\t<laugon>PENALTY</laugon>\n"
               << "\t\t<tolerance>0.01</tolerance>\n"
               << "\t\t<penalty>1</penalty>\n"
               << "\t\t<minaug>0</minaug>\n"
               << "\t\t<maxaug>10</maxaug>\n"
               << "\t\t<search_tolerance>0.0001</search_tolerance>\n"
               << "\t\t<max_traction>0</max_traction>\n"
               << "\t\t<snap_tol>0</snap_tol>\n"
               << "\t</contact>\n";
    } else if (type == "surface constraint") {
        stream << "\t<contact name=\"" << contact_name << "\" surface_pair=\"" << surface_pair_name
               << "\" type=\"surface constraint\">\n"
               << "\t\t<laugon>PENALTY</laugon>\n"
               << "\t\t<tolerance>0</tolerance>\n"
               << "\t\t<penalty>0</penalty>\n"
               << "\t\t<two_pass>0</two_pass>\n"
               << "\t</contact>\n";
    } else if (type == "tied-elastic") {
        stream << "\t<contact name=\"" << contact_name << "\" surface_pair=\"" << surface_pair_name
               << "\" type=\"tied-elastic\">\n"
               << "\t\t<laugon>PENALTY</laugon>\n"
               << "\t\t<tolerance>0.1</tolerance>\n"
               << "\t\t<gaptol>-1</gaptol>\n"
               << "\t\t<penalty>1</penalty>\n"
               << "\t\t<auto_penalty>0</auto_penalty>\n"
               << "\t\t<update_penalty>0</update_penalty>\n"
               << "\t\t<two_pass>0</two_pass>\n"
               << "\t\t<knmult>1</knmult>\n"
               << "\t\t<search_tol>0.01</search_tol>\n"
               << "\t\t<symmetric_stiffness>1</symmetric_stiffness>\n"
               << "\t\t<search_radius>1</search_radius>\n"
               << "\t\t<minaug>0</minaug>\n"
               << "\t\t<maxaug>10</maxaug>\n"
               << "\t</contact>\n";
    } else if (type == "tied-facet-on-facet") {
        stream << "\t<contact name=\"" << contact_name << "\" surface_pair=\"" << surface_pair_name
               << "\" type=\"tied-facet-on-facet\">\n"
               << "\t\t<laugon>PENALTY</laugon>\n"
               << "\t\t<tolerance>0.01</tolerance>\n"
               << "\t\t<penalty>1</penalty>\n"
               << "\t\t<minaug>0</minaug>\n"
               << "\t\t<maxaug>10</maxaug>\n"
               << "\t\t<search_tolerance>0.0001</search_tolerance>\n"
               << "\t\t<gap_offset>0</gap_offset>\n"
               << "\t</contact>\n";
    } else if (type == "tied-node-on-facet") {
        stream << "\t<contact name=\"" << contact_name << "\" surface_pair=\"" << surface_pair_name
               << "\" type=\"tied-node-on-facet\">\n"
               << "\t\t<laugon>PENALTY</laugon>\n"
               << "\t\t<tolerance>0.01</tolerance>\n"
               << "\t\t<penalty>1</penalty>\n"
               << "\t\t<minaug>0</minaug>\n"
               << "\t\t<maxaug>10</maxaug>\n"
               << "\t\t<search_tolerance>0.0001</search_tolerance>\n"
               << "\t\t<offset_shells>0</offset_shells>\n"
               << "\t\t<max_distance>0</max_distance>\n"
               << "\t\t<special>1</special>\n"
               << "\t\t<node_reloc>0</node_reloc>\n"
               << "\t</contact>\n";
    } else {
        std::runtime_error("contact type is not supported");
    }
}


} // namespace wmtk::components::internal