import Jama.Matrix;
import java.util.Arrays;

public class MainActivity {

    static double diametroEstacas = 0.5;
    static double comprimentoEstacas = 10;
    static String fckConcreto = "25";

    static double[][] esforcos = {{100}, {0}, {0}, {0}, {0}, {0}};

    /* Exemplo de estaqueamento com todas as estacas paralelas */
    static Estacas[] estaqueamento = new Estacas[3];

    /* Exemplo de estaqueamento plano em XY */
//    Estacas[] estaqueamento = new Estacas[3];

    /* Exemplo de estaqueamento plano em XZ */
//    public static Estacas[] estaqueamento = new Estacas[3];

    /*Exemplo de estaqueamento com simetria em relacao ao plano XY */
//    public static Estacas[] estaqueamento = new Estacas[5];

    /*Exemplo de estaqueamento com simetria em relacao ao plano XY */
//    public static Estacas[] estaqueamento = new Estacas[5];

    /*Exemplo de estaqueamento com simetria em relacao aos planos XY e XZ */
//    public static Estacas[] estaqueamento = new Estacas[8];

    /*Exemplo de estaqueamento com simetria em relacao ao eixo x */
//    public static Estacas[] estaqueamento = new Estacas[5];

    static double[] movElastico;
    static Matrix reacoesNormais;

    public static void main(String[] args) {

        AnalisesPreliminares auxiliar;

        /* Exemplo de estaqueamento com todas as estacas paralelas */
        estaqueamento[0] = new Estacas(1, 0, 0, 0, 0);
        estaqueamento[1] = new Estacas(1, 1, 0, 0, 0);
        estaqueamento[2] = new Estacas(1, -1, 0, 0, 0);
//        estaqueamento[3] = new Estacas(1, 0.4, 0.6, 0, 0);
//        estaqueamento[4] = new Estacas(1,0,0,0,0);

        /* Exemplo de estaqueamento plano em XY */
//        estaqueamento[0] = new Estacas(1, 0.3, 0, 10, 0);
//        estaqueamento[1] = new Estacas(1, 0, 0, 0, 0);
//        estaqueamento[2] = new Estacas(1, -0.3, 0, 10, 180);

        /* Exemplo de estaqueamento plano em XZ */
//        estaqueamento[0] = new Estacas(1, 0, 0.3, 10, 90);
//        estaqueamento[1] = new Estacas(1, 0, 0, 0, 0);
//        estaqueamento[2] = new Estacas(1, 0, -0.3, 10, 270);

        /*Exemplo de estaqueamento com simetria em relacao ao plano XY */
//        estaqueamento[0] = new Estacas(1, 0.4, 0.4, 10, 30);
//        estaqueamento[1] = new Estacas(1, 0, 0.4, 10, 90);
//        estaqueamento[2] = new Estacas(1, -0.4, 0.4, 10, 150);
//        estaqueamento[3] = new Estacas(1, -0.3, -0.3, 10, 225);
//        estaqueamento[4] = new Estacas(1, 0.3, -0.3, 10, 315);

        /*Exemplo de estaqueamento com simetria em relacao ao plano XY */
//        estaqueamento[0] = new Estacas(1, -0.3, -0.3, 10, 210);
//        estaqueamento[1] = new Estacas(1, -0.3, 0.3, 10, 150);
//        estaqueamento[2] = new Estacas(1, 0.3, 0, 10, 0);
//        estaqueamento[3] = new Estacas(1, 0.3, -0.5, 10, 330);
//        estaqueamento[4] = new Estacas(1, 0.3, 0.5, 10, 30);

        /*Exemplo de estaqueamento com simetria em relacao aos planos XY e XZ */
//        estaqueamento[0] = new Estacas(1, 0.5, 0.3, 10, 30);
//        estaqueamento[1] = new Estacas(1, -0.5, 0.3, 10, 150);
//        estaqueamento[2] = new Estacas(1, -0.5, -0.3, 10, 210);
//        estaqueamento[3] = new Estacas(1, 0.5, -0.3, 10, 330);
//        estaqueamento[4] = new Estacas(1, 0.2, 0, 10, 0);
//        estaqueamento[5] = new Estacas(1, -0.2, 0, 10, 180);
//        estaqueamento[6] = new Estacas(1, 0, 0.2, 10, 90);
//        estaqueamento[7] = new Estacas(1, 0, -0.2, 10, 270);

        /*Exemplo de estaqueamento com simetria em relacao ao eixo x */
//        estaqueamento[0] = new Estacas(1, 0.5, 0.3, 10, 300);
//        estaqueamento[1] = new Estacas(1, 0, 0, 0, 0);
//        estaqueamento[2] = new Estacas(1, -0.5, -0.3, 10, 120);
//        estaqueamento[3] = new Estacas(1, 0.4, 0.2, 12, 50);
//        estaqueamento[4] = new Estacas(1, -0.4, -0.2, 12, 230);

        auxiliar = new AnalisesPreliminares();

        char casoSimetria = auxiliar.getCasoSimetria();

        System.out.println(casoSimetria);

        auxiliar.calcularCaso(casoSimetria);
    }
}