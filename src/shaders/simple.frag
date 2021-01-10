out vec4 f_color;

in V_OUT
{
   vec3 position;
   vec3 normal;
   vec2 texture_coordinate;
} f_in;

uniform vec3 u_color;

uniform sampler2D u_texture;

void main()
{   
	//vec3 normal = normalize(f_in.normal);

    //f_color = vec4(f_in.texture_coordinate.xy, 0.0f, 1.0f);
    //f_color = vec4(u_color, 1.0);
    vec3 t_color = texture2D(u_texture, f_in.texture_coordinate.xy).rgb;
    //if(t_color == vec3(1,1,1))
      //  discard;
    f_color = vec4(t_color * u_color, 1.0);
}