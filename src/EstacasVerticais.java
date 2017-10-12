import Jama.Matrix;

public class EstacasVerticais extends AnalisesPreliminares{

    private Matrix matrizComponentesEstacasReduzida;
    private Matrix matrizRigidezReduzida;
    private Matrix matrizEsforcosExternos;

    private Matrix tensorTransformacao;

    private Matrix matrizComponentesEstacasRedTransformados;
    private Matrix matrizRigidezRedTransformada;
    private Matrix matrizEsforcosRedTransformados;
    private Matrix matrizMovElasticoRedTransformado;

    private double fi;
    private double yo;
    private double zo;

    /* Método para calcular os esforços normais de reação nas estacas: [N] = [v']transposta * [p'] */
    public Matrix calcularEsforcosNormais() {

        matrizComponentesEstacasReduzida = reduzirMatrizCompontesEstacas();
        matrizRigidezReduzida = reduzirMatrizRigidez();
        matrizEsforcosExternos = montarMatrizEsforcosReduzidos();

        acharNovasCoordenadas();
        tensorTransformacao = montarTensorTransformacao();

        matrizComponentesEstacasRedTransformados = calcularComponentesEstacasTransformados();
        matrizRigidezRedTransformada = calcularRigidezTransformada();
        matrizEsforcosRedTransformados = calcularEsforcosTransformados();
        matrizMovElasticoRedTransformado = calcularMovElasticoTransformado();

        calcularMovElastico();

        return (getMatrizRigidezEstacas().times(matrizComponentesEstacasRedTransformados.transpose())).times(matrizMovElasticoRedTransformado);
    }

    /* Método chamado para reduzir a matriz das componentes das estacas em caso de degeneração */
    private Matrix reduzirMatrizCompontesEstacas() {

        double[][] matriz = new double[3][MainActivity.estaqueamento.length];
        double[][] componentes = getMatrizComponentesEstacas().getArray();

        for (int i = 0; i < MainActivity.estaqueamento.length; i++) {

            matriz[0][i] = componentes[0][i];
            matriz[1][i] = componentes[4][i];
            matriz[2][i] = componentes[5][i];
        }

        return new Matrix(matriz);
    }

    /* Método chamado para reduzir a matriz de rigidez do estaqueamento em caso de degeneração */
    private Matrix reduzirMatrizRigidez() {

        double[][] matrizReduzida = new double[3][3];

        matrizReduzida[0][0] = getMatrizRigidez().getArray()[0][0];
        matrizReduzida[0][1] = getMatrizRigidez().getArray()[0][4];
        matrizReduzida[0][2] = getMatrizRigidez().getArray()[0][5];

        matrizReduzida[1][0] = getMatrizRigidez().getArray()[4][0];
        matrizReduzida[1][1] = getMatrizRigidez().getArray()[4][4];
        matrizReduzida[1][2] = getMatrizRigidez().getArray()[4][5];

        matrizReduzida[2][0] = getMatrizRigidez().getArray()[5][0];
        matrizReduzida[2][1] = getMatrizRigidez().getArray()[5][4];
        matrizReduzida[2][2] = getMatrizRigidez().getArray()[5][5];

        return new Matrix(matrizReduzida);
    }

    /* Método chamado para reduzir a matriz de esforços externos em caso de degeneração */
    private Matrix montarMatrizEsforcosReduzidos() {

        double[][] matriz = new double[3][1];

        matriz[0][0] = MainActivity.esforcos[0][0];
        matriz[1][0] = MainActivity.esforcos[4][0];
        matriz[2][0] = MainActivity.esforcos[5][0];

        return new Matrix(matriz);
    }

    /* Método para achar as coordenadas dos novos eixos coordenados (xo, yo), em relação ao antigo,
    e o ângulo de rotação fi, após a translação e rotação dos antigos eixos coordenados como
    aritifício para solucionar casos de estaqueamento apenas com estacas verticais. */
    private void acharNovasCoordenadas() {

        double[][] mS = matrizRigidezReduzida.getArray();

        if ((mS[0][0]*(mS[1][1]-mS[2][2]) + Math.pow(mS[0][2],2) + Math.pow(mS[0][1],2)) == 0) {

            fi = 0;

        } else {

            fi = (Math.atan((2 * (mS[0][0] * mS[1][2] - mS[0][1] * mS[0][2]) / (mS[0][0] * (mS[2][2] - mS[1][1]) + Math.pow(mS[0][1], 2) - Math.pow(mS[0][2], 2))))) / 2;
        }

        yo = (- mS[0][1] * Math.sin(fi) - mS[0][2] * Math.cos(fi))/mS[0][0];
        zo = (  mS[0][1] * Math.cos(fi) - mS[0][2] * Math.sin(fi))/mS[0][0];
    }

    /* Método para criar a matriz de trnasformação [T], ou tensor de transformação [T], referente aos
    casos de estaquemaneto plano em XY ou XZ retornando uma matriz 3 x 3 por apresentar degeneração */
    private Matrix montarTensorTransformacao() {

        double[][] tensorT = new double[3][3];

        tensorT[0][0] = 1;
        tensorT[0][1] = 0;
        tensorT[0][2] = 0;

        tensorT[1][0] = -zo;
        tensorT[1][1] = Math.cos(fi);
        tensorT[1][2] = -Math.sin(fi);

        tensorT[2][0] = yo;
        tensorT[2][1] = Math.sin(fi);
        tensorT[2][2] = Math.cos(fi);

        return new Matrix(tensorT);
    }

    /* Método para calcular a matriz de componentes de estaca transformados [p'], aplicando o tensor de
    transformação [T], retornando uma matriz 3 x n: [p'] = [T] * [p], por apresentar degeneração,
    onde n é número de estacas */
    private Matrix calcularComponentesEstacasTransformados() {

        return tensorTransformacao.times(matrizComponentesEstacasReduzida);
    }

    /* Método para calcular a matriz de rigidez transformada [S'], aplicando o tensor de transformação [T],
    retornando uma matriz 3 x 3, por apresentar degeneração: [S'] = [T] * [S] * [T]transposta */
    private Matrix calcularRigidezTransformada() {

        return tensorTransformacao.times(matrizRigidezReduzida.times(tensorTransformacao.transpose()));
    }

    /* Método para calcular a matriz de esforços transformados [F'], aplicando o tensor de transformação [T],
    retornando uma matraiz 3 x 1, por apresentar degeneração: [F'] = [T] * [F] */
    private Matrix calcularEsforcosTransformados() {

        return tensorTransformacao.times(matrizEsforcosExternos);
    }

    /* Método para calcular a matriz do movimento elástico transformado do bloco [v'], após
    a transformação da matriz de rigidez [S'] e da matriz do esforços externos [F'],
    retornando uma matriz 3 x 1, por apresentar degeneração: [v'] = [S' ^ -1] * [F'] */
    private Matrix calcularMovElasticoTransformado() {

        double[][] matriz = new double[3][1];

        matriz[0][0] = matrizEsforcosRedTransformados.getArray()[0][0] / matrizRigidezRedTransformada.getArray()[0][0];
        matriz[1][0] = matrizEsforcosRedTransformados.getArray()[1][0] / matrizRigidezRedTransformada.getArray()[1][1];
        matriz[2][0] = matrizEsforcosRedTransformados.getArray()[2][0] / matrizRigidezRedTransformada.getArray()[2][2];

        return new Matrix(matriz);
    }

    private void calcularMovElastico() {

        double[] matriz = new double[6];

        matriz[0] = (tensorTransformacao.transpose().times(matrizMovElasticoRedTransformado)).getArray()[0][0];
        matriz[1] = 0;
        matriz[2] = 0;
        matriz[3] = 0;
        matriz[4] = (tensorTransformacao.transpose().times(matrizMovElasticoRedTransformado)).getArray()[1][0];
        matriz[5] = (tensorTransformacao.transpose().times(matrizMovElasticoRedTransformado)).getArray()[2][0];

        MainActivity.movElastico = matriz;
    }

    public Matrix getMatrizComponentesEstacasReduzido() {
        return matrizComponentesEstacasReduzida;
    }

    public Matrix getMatrizRigidezReduzida() {
        return matrizRigidezReduzida;
    }

    public Matrix getMatrizEsforcosExternos() {
        return matrizEsforcosExternos;
    }

    public Matrix getTensorTransformacao() {
        return tensorTransformacao;
    }

    public Matrix getMatrizComponentesEstacasRedTransformados() {
        return matrizComponentesEstacasRedTransformados;
    }

    public Matrix getMatrizRigidezRedTransformada() {
        return matrizRigidezRedTransformada;
    }

    public Matrix getMatrizEsforcosRedTransformados() {
        return matrizEsforcosRedTransformados;
    }

    public Matrix getMatrizMovElasticoRedTransformado() {
        return matrizMovElasticoRedTransformado;
    }

}