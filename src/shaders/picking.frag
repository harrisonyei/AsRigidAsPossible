out uint FragColor;

void main()
{
    FragColor = gl_PrimitiveID + 1;
}